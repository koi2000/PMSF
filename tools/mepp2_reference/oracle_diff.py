#!/usr/bin/env python3
"""把候选连通性符号序列与 MEPP2 oracle fixture 对拍，报告**首个分歧点**。

P13.1 的判据是"同网格同量化 ⇒ 连通性符号序列一致"（不是字节一致 ——
熵编码器不同，字节必然不同）。本工具把这条判据变成可执行的检查。

用法:
  oracle_diff.py <oracle_fixture> <candidate>
  oracle_diff.py --selftest          # 用 fixture 自身对拍，应当 0 分歧

候选文件格式：每行一个符号（十进制整数），或用 `symbols a b c ...` 单行形式
（与 oracle dump 同格式）。两种都接受，且忽略 `#` 注释行。
"""
import sys


def load(path):
    syms = []
    layered = []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('symbols'):
            syms.extend(int(x) for x in line.split()[1:])
        else:
            for tok in line.split():
                syms.append(int(tok))
    return syms


def load_layers(path):
    import re
    out = []
    for line in open(path):
        m = re.match(r'# layer (\d+) symbols (\d+) vertices (\d+)', line.strip())
        if m:
            out.append((int(m.group(1)), int(m.group(2)), int(m.group(3))))
    return out


def main():
    if len(sys.argv) == 2 and sys.argv[1] == '--selftest':
        a = load(FIXTURE)
        b = load(FIXTURE)
        rc = compare(a, b, "fixture", "fixture")
        print("[selftest] 自比对应当 0 分歧 ⇒", "PASS" if rc == 0 else "FAIL")
        return rc
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    return compare(load(sys.argv[1]), load(sys.argv[2]), sys.argv[1], sys.argv[2])


FIXTURE = __file__.rsplit('/', 1)[0] + '/oracle/fandisk_q10_base19.txt'


def compare(a, b, na, nb):
    print(f"oracle    : {na}  ({len(a)} symbols)")
    print(f"candidate : {nb}  ({len(b)} symbols)")

    n = min(len(a), len(b))
    for i in range(n):
        if a[i] != b[i]:
            print(f"\n❌ 首个分歧 @ 符号 #{i}: oracle={a[i]} candidate={b[i]}")
            lo = max(0, i - 8)
            print(f"   oracle   [{lo}:{i+1}] = {a[lo:i+1]}")
            print(f"   candidate[{lo}:{i+1}] = {b[lo:i+1]}")
            # 定位到层（层内序号 → 层号）
            cum = 0
            for (li, nsym, nv) in load_layers(na):
                if cum + nsym > i:
                    print(f"   → 第 {li} 层内第 {i - cum} 个符号 "
                          f"(该层共 {nsym} 符号, 删 {nv} 顶点)")
                    break
                cum += nsym
            return 1

    if len(a) != len(b):
        who = "oracle" if len(a) > len(b) else "candidate"
        print(f"\n❌ 前 {n} 个符号一致，但长度不同：{who} 更长")
        print(f"   多出的开头: {(a if len(a) > len(b) else b)[n:n+16]}")
        return 1

    print(f"\n✅ 完全一致：{len(a)} 个符号逐位相同")
    return 0


if __name__ == '__main__':
    sys.exit(main())
