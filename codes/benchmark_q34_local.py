# -*- coding: utf-8 -*-
"""q3/q4 本地官方 Engine 基准：确定性场景、进程内传输和完整回放记录。

线程纪律（2026-09-13）：多线程 BLAS 的归约顺序随负载/线程调度变化，会被
策略决策阈值放大成 ±3% 的过程级虚拟时间漂移，使跨进程/跨批次的 A/B 对比
失效（实测同一配置漂移 -65~+36 s/任务）。本模块在导入 numpy 前把线程数
固定为 1，保证任意两次运行逐位可比；需要并行的实验应分进程串行执行。
"""
from __future__ import annotations
import os
for _thread_var in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                    'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
    os.environ.setdefault(_thread_var, '1')
import argparse, dataclasses, hashlib, importlib.metadata, importlib.util, json, math, platform, subprocess, sys, time
from pathlib import Path
from .local_simulator import Engine, SimError, generate_practice
from .protocol import RobotClient, decode
from .strategy import run_strategy
from .problem3_solution import Problem3Config, run_mission as run3
from .problem4_solution import Problem4Config, run_mission as run4

def _json(v):
    """递归转换为严格 JSON 值。"""
    if dataclasses.is_dataclass(v): return _json(dataclasses.asdict(v))
    if isinstance(v, dict): return {str(k): _json(x) for k,x in v.items()}
    if isinstance(v, (list,tuple,set)): return [_json(x) for x in v]
    if hasattr(v, 'item'): return _json(v.item())
    if isinstance(v, float) and not math.isfinite(v): return None
    return v

def generate_scenario(problem: int, seed: int) -> dict:
    """以 seed 派生固定 32 字节密钥，调用官方本地场景生成器。"""
    key = hashlib.sha256(f'q34-local-seed-v1:{problem}:{seed}'.encode()).digest()
    return generate_practice(problem, key)

class EngineTransport:
    """把 Engine 的动作映射成 RobotClient 所需的 (HTTP 状态, JSON 字节)。"""
    def __init__(self, scenario): self.engine = Engine(scenario)
    def __call__(self, path, body, timeout=5.0):
        """执行一次进程内协议请求，禁止向求解器暴露场景真值。"""
        try:
            p = decode(body)
            if path == '/enter':
                if self.engine.entered or self.engine.exited: return self._err(409, 'already_entered')
                self.engine.entered = True
                return self._ok(max_real_duration_s=1200, max_virtual_duration_s=360000,
                                 remaining_real_duration_s=1200)
            if path == '/exit':
                self.engine.exited = True; self.engine.stop_reason = 'user_exit'
                return self._ok(exit_reason='user_exit')
            pos, ch = p['position'], p['channel']; x, y = pos['x'], pos['y']
            if path == '/measure': extra = self.engine.measure(ch, x, y)
            elif path == '/clear': extra = self.engine.clear(ch, x, y)
            else: return self._err(404, 'unknown_path')
            return self._ok(**extra)
        except Exception as exc:
            return self._err(getattr(exc, 'status', 400), getattr(exc, 'code', type(exc).__name__))
    def _ok(self, **extra):
        return 200, json.dumps({'accepted': True, 'real_timestamp_ms': time.time_ns()//1_000_000,
                                'virtual_time_s': self.engine.virtual_time_s(), **extra}, ensure_ascii=False).encode()
    def _err(self, status, code):
        return status, json.dumps({'accepted': False, 'real_timestamp_ms': time.time_ns()//1_000_000,
                                   'virtual_time_s': self.engine.virtual_time_s(), 'error': code}).encode()

def _config(problem, overrides):
    """校验并构造对应问题配置。"""
    cls = Problem3Config if problem == 3 else Problem4Config
    names = {f.name for f in dataclasses.fields(cls)}
    bad = set(overrides) - names
    if bad: raise ValueError(f'unknown config fields: {sorted(bad)}')
    return cls(**overrides)

def _code_hashes(root=None):
    """记录参与运行的源码哈希，支持未提交修改追溯。"""
    root = Path(root) if root else Path(__file__).parent
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.glob('*.py'))}

def load_snapshot_package(path):
    """按包方式加载保存的算法快照，供严格 pristine 基线调用。"""
    path = Path(path).resolve()
    spec = importlib.util.spec_from_file_location('_q34_baseline', path/'__init__.py', submodule_search_locations=[str(path)])
    if spec is None or spec.loader is None: raise ImportError(f'cannot load snapshot: {path}')
    module = importlib.util.module_from_spec(spec); sys.modules[spec.name] = module; spec.loader.exec_module(module)
    return module

def _checks(record, scenario, transport):
    """独立 Engine 重放每个动作，核查反馈、时钟、路径、边界及完成证据。"""
    rec = record or {}
    channels, steps = rec.get('cleared_channels') or [], rec.get('steps') or []
    traj = rec.get('trajectory') or []
    truth = {j['channel'] for j in scenario['jammers']}
    checks = {}
    checks['channels_legal'] = all(type(c) is int and 1 <= c <= 20 for c in channels)
    checks['cleared_unique'] = len(channels) == len(set(channels))
    checks['cleared_truth_match'] = set(channels) <= truth
    try:
        checks['finite_coordinates'] = all(len(p) == 2 and all(
            math.isfinite(float(v)) and abs(float(v)) <= 2_000_000 for v in p) for p in traj)
        checks['trajectory_steps_match'] = (len(traj) == len(steps) + 1
            and math.dist(traj[0], (0., 0.)) < 1e-8
            and all(math.dist(traj[i + 1], (s['x'], s['y'])) < 1e-7 for i, s in enumerate(steps)))
        distance = sum(math.dist(a, b) for a, b in zip(traj, traj[1:]))
        checks['distance_match'] = math.isclose(float(rec['moved_distance_m']), distance,
                                               rel_tol=1e-9, abs_tol=1e-5)
    except (ValueError, TypeError, KeyError, IndexError):
        checks.setdefault('finite_coordinates', False)
        checks['trajectory_steps_match'] = checks['distance_match'] = False
    replay = Engine(scenario)
    checks['engine_replay_match'] = True
    checks['feedback_match'] = True
    checks['action_channels_legal'] = True
    for step in steps:
        try:
            channel, kind = step['channel'], step['kind']
            checks['action_channels_legal'] &= type(channel) is int and 1 <= channel <= 20
            if kind not in ('measure', 'clear'):
                raise ValueError('unknown action kind')
            response = getattr(replay, kind)(channel, step['x'], step['y'])
            checks['feedback_match'] &= response[kind + '_result'] == step['result']
            checks['engine_replay_match'] &= math.isclose(float(step['virtual_time_s']),
                float(replay.virtual_time_s()), rel_tol=0., abs_tol=1e-6)
        except (ValueError, TypeError, KeyError, IndexError, RuntimeError, SimError):
            checks['engine_replay_match'] = checks['feedback_match'] = False
            break
    checks['replayed_clears_match'] = set(channels) == replay.cleared
    try:
        end = float(rec['end_virtual_time_s'])
        checks['virtual_time_match'] = (float(rec['start_virtual_time_s']) == 0.
            and math.isclose(end, float(replay.virtual_time_s()), rel_tol=0., abs_tol=1e-6)
            and math.isclose(end, float(transport.engine.virtual_time_s()), rel_tol=0., abs_tol=1e-6))
    except (KeyError, TypeError, ValueError):
        checks['virtual_time_match'] = False
    checks['planner_clean'] = not rec.get('planner_errors', 0) and not rec.get('inconsistent_channels', [])
    completed = rec.get('stop_reason') in ('all_channels_resolved', 'cleared_limit')
    checks['complete_valid'] = completed and rec.get('completed', completed) and set(channels) == truth
    checks['all_constraints_ok'] = all(checks.values())
    return checks

def run_case(problem, seed, overrides, source_dir=None):
    """运行单个可复现实例并返回原始记录与审计信息。"""
    scenario = generate_scenario(problem, seed); transport = EngineTransport(scenario)
    client = RobotClient('local-benchmark', transport)
    if source_dir:
        load_snapshot_package(source_dir)
        module = importlib.import_module(f'_q34_baseline.problem{problem}_solution')
        cfg = getattr(module, f'Problem{problem}Config')(**overrides)
        mission = module.run_mission
    else:
        cfg = _config(problem, overrides)
        mission = run3 if problem == 3 else run4
    solver = lambda ctx: mission(ctx, cfg)
    error = None; result = {}; start = time.perf_counter()
    try: result = run_strategy(client, solver, problem=problem)
    except Exception as exc: error = {'type': type(exc).__name__, 'message': str(exc)}
    wall = time.perf_counter()-start; rec = result.get('algorithm_result') or {}
    cleared = sorted(client.state.cleared_channels)
    return {'seed': seed, 'problem': problem, 'scenario': _json(scenario), 'config': _json(cfg),
            'record': _json(rec), 'runner_result': _json(result), 'wall_time_s': wall,
            'virtual_time_s': transport.engine.virtual_time_s(), 'cleared': len(cleared),
            'true_total': scenario['jammer_count'], 'completion_rate': len(cleared)/scenario['jammer_count'],
            'checks': _checks(rec, scenario, transport), 'error': error}

def summary(cases):
    """主指标为每个场景 T/实际清除数的均值；任一空清除则不报告有限主指标。"""
    return {
        'mean_virtual_time_s': sum(x['virtual_time_s'] for x in cases) / len(cases),
        'mean_wall_time_s': sum(x['wall_time_s'] for x in cases) / len(cases),
        'mean_time_per_cleared_source_s': (sum(x['virtual_time_s'] / x['cleared'] for x in cases) / len(cases)
                                          if all(x['cleared'] for x in cases) else None),
        'completion_rate': sum(x['completion_rate'] for x in cases) / len(cases),
        'constraints_passed': sum(bool(x['checks']['all_constraints_ok']) for x in cases),
        'cases': len(cases),
    }


def main(argv=None):
    """解析 CLI，逐例执行并写入单一 UTF-8 JSON 文件。"""
    p=argparse.ArgumentParser(); p.add_argument('--problem',type=int,choices=(3,4),required=True)
    p.add_argument('--seeds',type=int,nargs='+',required=True); p.add_argument('--overrides',default='{}')
    p.add_argument('--source-dir', type=Path, help='载入本轮保存的原始算法包快照')
    p.add_argument('--output',type=Path,required=True); a=p.parse_args(argv)
    raw=a.overrides
    if Path(raw).is_file(): raw=Path(raw).read_text(encoding='utf-8')
    overrides=json.loads(raw)
    if not isinstance(overrides, dict):
        raise ValueError('--overrides 必须是 JSON 对象')
    pre_hashes=_code_hashes(a.source_dir); cases=[]
    payload={'schema':'q34-local-benchmark-v1','problem':a.problem,'seeds':a.seeds,
             'overrides':overrides,'code_hashes':pre_hashes,'git_hash':subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True).stdout.strip(),
             'python':sys.version,'platform':platform.platform(),'dependencies':{d:importlib.metadata.version(d) for d in ('numpy',)},
             'source_dir': str(a.source_dir) if a.source_dir else None, 'cases':cases}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    for seed in a.seeds:
        cases.append(run_case(a.problem, seed, overrides, a.source_dir))
        payload['summary'] = summary(cases)
        a.output.write_text(json.dumps(_json(payload),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
        print(json.dumps({'seed': seed, **payload['summary']},ensure_ascii=False), flush=True)
    return payload
if __name__ == '__main__': main()
