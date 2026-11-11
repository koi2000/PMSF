#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# impl/MMSE/mmse_cq_probe.sh
#
# P3 probe for the `mmse_cq` codec (MMSE paper §2 on the valence-conquest
# backbone): q12 horse19k, configurable rounds, columnar backend.
#
# For each rounds value it:
#   1. compresses with algorithm=`mmse_cq` (default) and algorithm=`conquest`
#      (reference backbone) and records artifact bytes/bpv/md5 for both,
#   2. decodes `mmse_cq`, checks the six MeshCompare flags + quality metrics,
#   3. compares the `mmse_cq` payload md5 against conquest's — P3-B1 must be
#      byte-identical (H3 = barycentric == engine default); P3-B2 will differ by
#      exactly the MMSE geometry changes,
#   4. records the hard-cap (22.10 bpv = 54,838 B) / ideal (20.40 = 50,620 B)
#      margins.
#
# Usage:
#   impl/MMSE/mmse_cq_probe.sh [out_dir] [rounds...]
# Env: BUILD_DIR, MESH, ALGO (default mmse_cq), REF_ALGO (default conquest)

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
COMPRESS_BIN="${BUILD_DIR}/PMSFCompress"
DECOMPRESS_BIN="${BUILD_DIR}/PMSFDecompress"
COMPARE_BIN="${BUILD_DIR}/MeshCompare"
SURFACE_BIN="${BUILD_DIR}/SurfaceMetrics"
MESH="${MESH:-${ROOT_DIR}/datasets/PaperModels/horse19k.off}"
ALGO="${ALGO:-mmse_cq}"
REF_ALGO="${REF_ALGO:-conquest}"
OUTDIR="${1:-${ROOT_DIR}/build/phase1/mmse_cq}"
shift || true
ROUNDS_LIST=("$@")
[[ ${#ROUNDS_LIST[@]} -gt 0 ]] || ROUNDS_LIST=(4 5 6 10 16)

NV=19851
CAP_BYTES="$(python3 -c "print(int(22.10*${NV}/8))")"
IDEAL_BYTES="$(python3 -c "print(int(20.40*${NV}/8))")"
CSV="${OUTDIR}/mmse_cq_probe.csv"

for bin in "${COMPRESS_BIN}" "${DECOMPRESS_BIN}" "${COMPARE_BIN}" "${SURFACE_BIN}"; do
    [[ -x "${bin}" ]] || { echo "missing binary ${bin}; build first" >&2; exit 3; }
done
[[ -f "${MESH}" ]] || { echo "missing mesh ${MESH}" >&2; exit 3; }
mkdir -p "${OUTDIR}"
OUTDIR="$(cd "${OUTDIR}" && pwd)"

printf '%s\n' \
"rounds,algo,artifact_bytes,bpv,hard_cap_margin_B,ideal_cap_margin_B,ref_bytes,ref_bpv,payload_md5,ref_payload_md5,payload_md5_match,decode_rc,counts_ok,connectivity_ok,topology_ok,nearest_geometry_ok,manifold_profile_ok,coordinate_remap_complete,rms_max,mse_sym" \
> "${CSV}"

make_cfg() { # dir rounds algo
    python3 - "$1" "$2" "$3" "${MESH}" <<'PYEOF'
import json, sys
d, rounds, algo, mesh = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
extra = {
    "PMSF_V2_COMPACT": "1",
    "PMSF_SYMBOL_BLOB_ZLIB": "1",
    "PMSF_DEFAULT_QUANT_BITS": "12",
    "PMSF_POOL_V": "100000",
    "PMSF_POOL_H": "480000",
    "PMSF_POOL_F": "200000",
    "PMSF_REPAIR_DEGENERATE": "1",
}
base = {
    "algorithm": algo, "backend": "columnar", "rounds": rounds,
    "quantization": True, "prediction": False, "segmentation": False,
    "adjustment": False, "compression": True,
    "input": mesh, "output": d + "/art.parquet", "extra": extra,
}
json.dump(base, open(d + "/c.json", "w"), indent=2)
json.dump({"algorithm": algo, "backend": "columnar",
           "input": d + "/art", "output": d + "/art.dec.off"},
          open(d + "/d.json", "w"))
PYEOF
}

fail=0
for r in "${ROUNDS_LIST[@]}"; do
    dir="${OUTDIR}/r${r}"
    refdir="${dir}/ref"
    mkdir -p "${dir}" "${refdir}"
    make_cfg "${dir}" "${r}" "${ALGO}"
    make_cfg "${refdir}" "${r}" "${REF_ALGO}"

    PMSF_BASE_EDGEBREAKER_NATIVE=1 "${COMPRESS_BIN}" "${dir}/c.json" > "${dir}/c.log" 2>&1
    c_rc=$?
    PMSF_BASE_EDGEBREAKER_NATIVE=1 "${COMPRESS_BIN}" "${refdir}/c.json" > "${refdir}/c.log" 2>&1

    bytes=$(( $(stat -c '%s' "${dir}/art_compact.bin" 2>/dev/null || echo 0) + $(stat -c '%s' "${dir}/art_symbols.parquet" 2>/dev/null || echo 0) ))
    refbytes=$(( $(stat -c '%s' "${refdir}/art_compact.bin" 2>/dev/null || echo 0) + $(stat -c '%s' "${refdir}/art_symbols.parquet" 2>/dev/null || echo 0) ))
    bpv=""; refbpv=""
    [[ "${bytes}" -gt 0 ]] && bpv="$(python3 -c "print(f'{8.0*${bytes}/${NV}:.6f}')")"
    [[ "${refbytes}" -gt 0 ]] && refbpv="$(python3 -c "print(f'{8.0*${refbytes}/${NV}:.6f}')")"
    margin=$((CAP_BYTES - bytes)); imargin=$((IDEAL_BYTES - bytes))
    md5p="$(md5sum "${dir}/art_symbols.parquet" 2>/dev/null | cut -d' ' -f1)"
    md5r="$(md5sum "${refdir}/art_symbols.parquet" 2>/dev/null | cut -d' ' -f1)"
    if [[ -n "${md5p}" && "${md5p}" == "${md5r}" ]]; then md5match=1; else md5match=0; fi

    "${DECOMPRESS_BIN}" "${dir}/d.json" > "${dir}/d.log" 2>&1
    d_rc=$?
    flags=",,,,,,"
    rms=""; mse=""
    if [[ ${d_rc} -eq 0 && -f "${dir}/art.dec.off" ]]; then
        "${COMPARE_BIN}" "${MESH}" "${dir}/art.dec.off" --tol 0.001 > "${dir}/compare.log" 2>&1
        flags="$(python3 - "${dir}/compare.log" <<'PYEOF'
import sys
want = ["counts_ok","connectivity_ok","topology_ok","nearest_geometry_ok",
        "manifold_profile_ok","coordinate_remap_complete"]
vals = {k: "" for k in want}
for line in open(sys.argv[1], errors="replace"):
    if "=" in line:
        k, v = line.strip().split("=", 1)
        if k.strip() in vals:
            vals[k.strip()] = v.strip()
print(",".join(vals[k] for k in want))
PYEOF
)"
        "${SURFACE_BIN}" "${MESH}" "${dir}/art.dec.off" --samples 5000 --json > "${dir}/surface.json" 2>/dev/null
        read -r rms mse < <(python3 - "${dir}/surface.json" <<'PYEOF'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
    print(d.get('rms_max', ''), d.get('mse_sym', ''))
except Exception:
    print("", "")
PYEOF
)
    fi
    printf '%s\n' "${r},${ALGO},${bytes},${bpv},${margin},${imargin},${refbytes},${refbpv},${md5p},${md5r},${md5match},${d_rc},${flags},${rms},${mse}" >> "${CSV}"
    echo "[mmse-cq] r=${r} ${ALGO}: ${bytes} B ${bpv} bpv (margin ${margin} B) decode_rc=${d_rc} flags=${flags} md5_match_conquest=${md5match}"
    [[ ${c_rc} -ne 0 ]] && fail=1
    [[ ${d_rc} -ne 0 ]] && fail=1
done

echo "[mmse-cq] CSV: ${CSV}"
exit ${fail}
