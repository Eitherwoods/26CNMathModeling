"""Go PE 二进制（jammers-simulator.exe）符号表解析与标注反汇编工具。

用法（在 tester/ 目录下）：
    python reverse/parse_pclntab.py symbols                 # 生成 symbols/ 两个文件
    python reverse/parse_pclntab.py dis GeneratePractice    # 按符号名子串反汇编并标注
    python reverse/parse_pclntab.py dis-all                 # 重建 disasm/ 与 annotated/

产物说明见 NOTES.md。Python 3.9+，无第三方依赖；dis 相关子命令需要 PATH 里有 objdump。
"""
import bisect
import json
import os
import re
import struct
import subprocess
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))          # tester/reverse
TESTER_DIR = os.path.dirname(BASE_DIR)                          # tester
EXE = os.path.join(TESTER_DIR, "jammers-simulator.exe")
# objdump 对中文路径会在 banner 里按本地代码页回显，统一在 TESTER_DIR 下用相对路径调用
EXE_ARG = "jammers-simulator.exe"
SYMBOLS_DIR = os.path.join(os.path.dirname(__file__), "symbols")
DISASM_DIR = os.path.join(os.path.dirname(__file__), "disasm")
ANNOTATED_DIR = os.path.join(os.path.dirname(__file__), "annotated")

IMAGE_BASE = 0x140000000
TEXT_RVA = 0x1000
TEXT_BASE = IMAGE_BASE + TEXT_RVA          # 0x140001000：functab entryoff 的基址
PCHeader_MAGIC = b"\xf1\xff\xff\xff"       # Go 1.18+ pclntab


def load_exe():
    with open(EXE, "rb") as f:
        return f.read()


def parse_sections(data):
    """返回 [(va, vsize, raw, rsize), ...]，用于 VA→文件偏移。"""
    pe_off = struct.unpack_from("<I", data, 0x3C)[0]
    nsec = struct.unpack_from("<H", data, pe_off + 6)[0]
    opt = struct.unpack_from("<H", data, pe_off + 20)[0]
    out = []
    for k in range(nsec):
        o = pe_off + 24 + opt + 40 * k
        vsize, va, rsize, raw = struct.unpack_from("<IIII", data, o + 8)
        out.append((va, vsize, raw, rsize))
    return out


def va2off(secs, va):
    rva = va - IMAGE_BASE
    for sva, svs, raw, rsz in secs:
        if sva <= rva < sva + max(svs, rsz):
            return raw + (rva - sva)
    return None


def read_va(secs, data, va, n):
    off = va2off(secs, va)
    return data[off:off + n] if off is not None else b""


def parse_pclntab(data):
    """扫描并解析 pclntab，返回 (entries, entry2name)。entry 为相对 TEXT_BASE 的偏移。"""
    cands = []
    pos = 0
    while True:
        i = data.find(PCHeader_MAGIC, pos)
        if i < 0:
            break
        pos = i + 1
        pad1, pad2, min_lc, ptr_size = data[i + 4], data[i + 5], data[i + 6], data[i + 7]
        if pad1 or pad2 or min_lc not in (1, 2, 4) or ptr_size != 8:
            continue
        fields = struct.unpack_from("<8Q", data, i + 8)
        funcname_off, pcln_off = fields[3], fields[7]
        if pcln_off >= len(data):
            continue
        nfunc = fields[0]
        if not 0 < nfunc < 10_000_000:
            continue
        # funcnametab 应紧跟可读符号名
        preview = data[i + funcname_off:i + funcname_off + 64]
        if not preview or not all(32 <= b < 127 or b == 0 for b in preview[:40]):
            continue
        names, ok, prev = [], True, -1
        for k in range(nfunc):
            entryoff, funcoff = struct.unpack_from("<II", data, i + pcln_off + 8 * k)
            if entryoff < prev:
                ok = False
                break
            prev = entryoff
            _, name_off = struct.unpack_from("<Ii", data, i + pcln_off + funcoff)
            ns = i + funcname_off + name_off
            end = data.find(b"\x00", ns, ns + 300)
            if end < 0:
                names.append(b"<?>\n")
            else:
                names.append(data[ns:end])
        if ok and len(names) > 1000:
            cands.append((i, names))
    if not cands:
        raise RuntimeError("pclntab not found")
    i, names = cands[0]
    # functab 重新走一遍拿 entryoff（与 names 一一对应）
    funcname_off, pcln_off = struct.unpack_from("<8Q", data, i + 8)[3], \
        struct.unpack_from("<8Q", data, i + 8)[7]
    nfunc = len(names)
    entry2name = {}
    for k in range(nfunc):
        entryoff, funcoff = struct.unpack_from("<II", data, i + pcln_off + 8 * k)
        _, name_off = struct.unpack_from("<Ii", data, i + pcln_off + funcoff)
        ns = i + funcname_off + name_off
        end = data.find(b"\x00", ns, ns + 300)
        if end > 0:
            entry2name[entryoff] = data[ns:end].decode("utf-8", "replace")
    return entry2name


def cmd_symbols():
    data = load_exe()
    entry2name = parse_pclntab(data)
    os.makedirs(SYMBOLS_DIR, exist_ok=True)
    with open(os.path.join(SYMBOLS_DIR, "entry2name.json"), "w", encoding="utf-8") as f:
        json.dump({hex(k): v for k, v in sorted(entry2name.items())}, f,
                  ensure_ascii=False, indent=0)
    with open(os.path.join(SYMBOLS_DIR, "all_symbols.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(entry2name[k] for k in sorted(entry2name)))
    print("functions:", len(entry2name), "->", SYMBOLS_DIR)


def _dump_function(data, secs, entry2name, entries, name):
    e = None
    for k, v in entry2name.items():
        if v == name:
            e = k
            break
    if e is None:
        raise SystemExit("symbol not found: " + name)
    j = bisect.bisect_right(entries, e)
    size = (entries[j] - e) if j < len(entries) else 0x400
    va = TEXT_BASE + e
    out = subprocess.run(["objdump", "-d", "-M", "intel",
                          f"--start-address={va:#x}", f"--stop-address={va + size:#x}",
                          EXE_ARG], capture_output=True, cwd=TESTER_DIR).stdout.decode("utf-8", "replace")
    keys = sorted(entry2name)
    lines = []
    for idx, ln in enumerate(out.splitlines()):
        m = re.search(r"\bcall\s+0x([0-9a-f]+)", ln)
        if m:
            t = int(m.group(1), 16) - TEXT_BASE
            k = bisect.bisect_right(keys, t) - 1
            if k >= 0 and t in entry2name:
                ln = ln.rstrip() + "   <" + entry2name[t] + ">"
        m = re.search(r"lea\s+\w+,\[rip\+0x[0-9a-f]+\]\s+#\s+0x([0-9a-f]+)", ln)
        if m:
            a = int(m.group(1), 16)
            ctx = " ".join(out.splitlines()[idx + 1:idx + 4])
            lm = re.search(r"mov\s+e[a-z]{2},(0x[0-9a-f]+)", ctx)
            if lm and 1 <= int(lm.group(1), 16) <= 60:
                s = read_va(secs, data, a, int(lm.group(1), 16))
                if s and all(32 <= c < 127 or c == 0 for c in s):
                    ln = ln.rstrip() + '   STR="%s"' % s.rstrip(b"\x00").decode()
            if "STR" not in ln:
                q1, q2 = struct.unpack("<QQ", read_va(secs, data, a, 16))
                if IMAGE_BASE <= q1 < 0x141200000 and 0 < q2 < 80:
                    s = read_va(secs, data, q1, q2)
                    if s and all(32 <= c < 127 for c in s):
                        ln = ln.rstrip() + '   STRHDR="%s"' % s.decode()
        fm = re.search(r"(?:movsd|comisd|ucomisd|subsd|divsd)\s+\S+,QWORD PTR "
                       r"\[rip\+0x[0-9a-f]+\]\s+#\s+0x([0-9a-f]+)", ln)
        if fm:
            d = struct.unpack("<d", read_va(secs, data, int(fm.group(1), 16), 8))[0]
            ln = ln.rstrip() + "   F64=%r" % d
        lines.append(ln.rstrip())
    return "\n".join(lines)


def cmd_dis(pattern):
    data = load_exe()
    secs = parse_sections(data)
    entry2name = parse_pclntab(data)
    entries = sorted(entry2name)
    for name in (v for v in entry2name.values() if pattern in v):
        print(_dump_function(data, secs, entry2name, entries, name))


def cmd_dis_all():
    data = load_exe()
    secs = parse_sections(data)
    entry2name = parse_pclntab(data)
    entries = sorted(entry2name)
    os.makedirs(DISASM_DIR, exist_ok=True)
    os.makedirs(ANNOTATED_DIR, exist_ok=True)
    name2entry = {}
    for k, v in entry2name.items():
        name2entry.setdefault(v, k)
    targets = sorted(v for v in entry2name.values()
                     if v.startswith(("jammers/client/internal/scenario.",
                                      "jammers/client/internal/bearingnoise.",
                                      "jammers/client/internal/simcore.",
                                      "jammers/client/internal/testsession.",
                                      "jammers/client/internal/robotapi.")))
    for name in targets:
        e = name2entry[name]
        j = bisect.bisect_right(entries, e)
        end = entries[j] if j < len(entries) else e + 0x400
        raw = subprocess.run(["objdump", "-d", "-M", "intel",
                              f"--start-address={TEXT_BASE + e:#x}",
                              f"--stop-address={TEXT_BASE + end:#x}",
                              EXE_ARG], capture_output=True, cwd=TESTER_DIR).stdout.decode("utf-8", "replace")
        safe = re.sub(r"[^\w.\-]", "_", name.split("/")[-1])
        with open(os.path.join(DISASM_DIR, safe + ".asm"), "w", encoding="utf-8") as f:
            f.write(raw)
        with open(os.path.join(ANNOTATED_DIR, safe + "_ann.asm"), "w", encoding="utf-8") as f:
            f.write(_dump_function(data, secs, entry2name, entries, name))
        print("dumped", name)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    if sys.argv[1] == "symbols":
        cmd_symbols()
    elif sys.argv[1] == "dis" and len(sys.argv) > 2:
        cmd_dis(sys.argv[2])
    elif sys.argv[1] == "dis-all":
        cmd_dis_all()
    else:
        print(__doc__)
        sys.exit(1)
