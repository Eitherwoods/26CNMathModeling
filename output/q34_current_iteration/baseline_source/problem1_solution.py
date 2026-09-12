# -*- coding: utf-8 -*-
"""复现思路文档中的两个人工验证案例，并支持用户 JSON 观测输入。"""
import argparse
import json
from pathlib import Path
import numpy as np
from .base_models import locate, bearing_halfplanes
from .config import PROBLEM1_OUTPUT_DIR, PROBLEM1_FIG_DIR, fmt6
from .plotting import plot_region


def example_data():
    """按文档未舍入真角生成五点观测；返回人工数据而非比赛观测。"""
    truth = np.array([300., 400.])
    stations = np.array([[-400, 0], [0, -300], [1000, 0], [900, 1000], [-300, 1000]])
    angles = np.rad2deg(np.arctan2(*(truth - stations)[:, ::-1].T)) % 360
    observations = np.column_stack((stations, (angles + [0.30, -0.45, 0.65, -0.20, 0.50]) % 360))
    return observations, truth


def triangle_data():
    """返回由三个合法 2 度角域形成正三角形的确定性反例。"""
    root = np.sqrt(3)
    return np.array([[-300, 0, 1], [160, -150 * root, 121],
                     [155, 155 * root, 241]]), np.array([5, 5 * root / 3])


def serialize_result(result):
    """转换结果为标准 JSON；无界直径用字符串表示，避免非标准 Infinity。"""
    return {'status': result.status, 'vertices_m': result.vertices.tolist(),
            'diameter_m': 'infinity' if result.status == 'unbounded' else result.diameter,
            'endpoints_m': result.endpoints.tolist()}


def main():
    """运行人工案例或 --input 指定的 m×3 JSON 数组，并保存数值和图形。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, help='每行 [x, y, 示向度] 的 JSON 数组')
    args = parser.parse_args()
    PROBLEM1_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    if args.input:
        observations = json.loads(args.input.read_text(encoding='utf-8'))
        payload = serialize_result(locate(observations))
        name = 'problem1_custom'
    else:
        payload = {'data_source': '思路文档人工构造，仅用于算法验证，不是模拟器成绩'}
        for case, factory in [('five_stations', example_data), ('triangle', triangle_data)]:
            observations, truth = factory()
            result = locate(observations)
            entry = serialize_result(result)
            a, b = bearing_halfplanes(observations)
            entry.update(observations=observations.tolist(), truth_m=truth.tolist(),
                         truth_max_residual_m=float(np.max(a @ truth - b)),
                         vertex_max_residual_m=float(np.max(a @ result.vertices.T - b[:, None])))
            midpoint = result.endpoints.mean(axis=0)
            entry['diameter_circle_max_excess_m'] = float(
                np.max(np.linalg.norm(result.vertices - midpoint, axis=1)) - result.diameter / 2)
            payload[case] = entry
            plot_region(observations, result, truth, PROBLEM1_FIG_DIR, case, case == 'triangle')
            print(f'{case}: {result.status}, 顶点数 {len(result.vertices)}, 直径 {fmt6(result.diameter)} m')
        name = 'problem1_results'
    target = PROBLEM1_OUTPUT_DIR / f'{name}.json'
    target.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(target)


if __name__ == '__main__':
    main()
