#!/usr/bin/env bash
# tools/mepp2_reference/oracle_dump.sh —— 生成 P13 用的 MEPP2 oracle 连通性符号流。
#
# MEPP2 是**开发期 oracle**（A/B 对拍基准），不是运行时依赖。
# 用「影子 include」给参考实现加一个 env 门控 dump：reference/MEPP2 保持只读。
#
# 用法： tools/mepp2_reference/oracle_dump.sh <输出目录> <mesh.off> <maxV> <Qbits> [oracle.txt]
set -euo pipefail

OUT="${1:?用法: oracle_dump.sh <输出目录> <mesh.off> <maxV> <Qbits> [oracle.txt]}"
MESH="$(cd "$(dirname "${2:?}")" && pwd)/$(basename "$2")"   # 必须绝对：
#   脚本下面会 cd 进 MEPP2，相对路径会失效（踩过）。
MAXV="${3:?}"; QBITS="${4:?}"
DUMP="${5:-${OUT}/oracle.txt}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MEPP2="${ROOT}/reference/MEPP2"
CV="${MEPP2}/FEVV/Filters/Generic/Manifold/Compression_Valence"
SHADOW="${OUT}/shadow/FEVV/Filters/Generic/Manifold/Compression_Valence"

mkdir -p "${OUT}" "${SHADOW}"
# 仓库自带的 External/CGAL-5.2.1 是被裁剪的子集（缺 CGAL/Cartesian.h），
# 系统 CGAL 5.6 与参考实现不兼容 ⇒ 用系统的 5.5.1 + cgal52_compat.h。
[[ -e "${OUT}/inc/CGAL" ]] || { mkdir -p "${OUT}/inc"; ln -sfn /usr/include/CGAL_5.5 "${OUT}/inc/CGAL"; }

# 影子 = 参考实现的副本 + 一处 env 门控 dump（脚本每次重放，保证与源码同步）。
# .h 与 .inl **都要**影子化：.h 用引号包含 .inl，引号包含优先同目录。
# 且 `-I` 必须指向 shadow 的**根**（其下才是 FEVV/... 路径），指到叶子目录
# 会静默失效 —— 且因为产物与未打补丁构建逐字节相同，**表面上完全看不出来**。
# 验证影子确实生效的办法： strings <binary> | grep PMSF_MEPP2_ORACLE_DUMP 非空。
cp "${CV}/Compression_Valence_Component.h" "${SHADOW}/"
python3 "${HERE}/oracle_patch.py" "${CV}/Compression_Valence_Component.inl" \
        "${SHADOW}/Compression_Valence_Component.inl"

cd "${MEPP2}"
g++ -std=c++17 -O2 -include boost/unordered_map.hpp \
    -include "${HERE}/cgal52_compat.h" \
    -I"${OUT}/shadow" -I"${CV}" -I. -I"${OUT}/inc" -IExternal/CImg/CImg-2.4.2 \
    -o "${OUT}/tcv_oracle" \
    Testing/CGAL/Polyhedron/test_compression_valence_polyhedron.cpp \
    -lgmp -lmpfr -lboost_filesystem -lboost_system

PMSF_MEPP2_ORACLE_DUMP="${DUMP}" "${OUT}/tcv_oracle" "${MESH}" "${OUT}/out.obj" \
    -withCompr -maxV "${MAXV}" -Qbits "${QBITS}" > "${OUT}/enc.log" 2>&1

echo "[oracle] dump → ${DUMP}"
awk '/^symbols/{print "[oracle] symbol count = " NF-1}' "${DUMP}"
grep "Total size" "${OUT}/enc.log" || true
