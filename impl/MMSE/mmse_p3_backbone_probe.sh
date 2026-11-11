#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# impl/MMSE/mmse_p3_backbone_probe.sh
#
# P3 feasibility spike (task-15 / WS-B4, 2026-10-22 round 2):
# measures the framework's **valence-conquest backbone** (the paper's own
# connectivity + decimation algorithm, ref [2] Alliez & Desbrun) on the MMSE
# paper model (horse19k, q12) as the candidate foundation for P3.
#
# It does NOT implement MMSE-on-conquest yet (that is the P3-B round); it only
# establishes the achievable floor/ceil of the backbone:
#   * rounds (layer-count) sweep of the conquest codec q12 horse19k,
#   * artifact total bytes + bpv + hard-cap (22.10 bpv) margin,
#   * component split of the CQST blob (connectivity symbols / geometry deltas /
#     fixed headers) parsed straight from the stored symbol blob,
#   * decode validity (MeshCompare six flags) + quality metrics,
#   * comparison against the current MMSE canonical row (62,633 B / 25.241247).
#
# Usage:
#   impl/MMSE/mmse_p3_backbone_probe.sh [out_dir]
#
# Env: BUILD_DIR, MESH, ROUNDS_LIST, CONQUEST_CFG (default configs/algorithms/RD.json,
#      which is the repo's algorithm=conquest config), EB_NATIVE (default 1).

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
COMPRESS_BIN="${BUILD_DIR}/PMSFCompress"
DECOMPRESS_BIN="${BUILD_DIR}/PMSFDecompress"
COMPARE_BIN="${BUILD_DIR}/MeshCompare"
SURFACE_BIN="${BUILD_DIR}/SurfaceMetrics"
MESH="${MESH:-${ROOT_DIR}/datasets/PaperModels/horse19k.off}"
CONQUEST_CFG="${CONQUEST_CFG:-${ROOT_DIR}/configs/algorithms/RD.json}"
OUTDIR="${1:-${ROOT_DIR}/build/phase1/mmse_p3_backbone}"
ROUNDS_LIST="${ROUNDS_LIST:-3 4 5 6 7 8 9 10 12 14 16 18 20 22 24}"
EB_NATIVE="${EB_NATIVE:-1}"
NV=19851
HARD_CAP_BPV=22.10
IDEAL_CAP_BPV=20.40
BASELINE_BYTES=62633
BASELINE_BPV=25.241247
CSV="${OUTDIR}/mmse_p3_backbone_probe.csv"

for bin in "${COMPRESS_BIN}" "${DECOMPRESS_BIN}" "${COMPARE_BIN}" "${SURFACE_BIN}"; do
    [[ -x "${bin}" ]] || { echo "missing binary ${bin}; build first" >&2; exit 3; }
done
[[ -f "${MESH}" && -f "${CONQUEST_CFG}" ]] || { echo "missing mesh/config" >&2; exit 3; }

mkdir -p "${OUTDIR}"
OUTDIR="$(cd "${OUTDIR}" && pwd)"

CAP_BYTES="$(python3 -c "print(int(${HARD_CAP_BPV}*${NV}/8))")"
IDEAL_BYTES="$(python3 -c "print(int(${IDEAL_CAP_BPV}*${NV}/8))")"
PYSPLIT="${ROOT_DIR}/scripts/warehouse/.venv/bin/python"
[[ -x "${PYSPLIT}" ]] || PYSPLIT="python3"

printf '%s\n' \
"rounds,compact_B,symbols_parquet_B,artifact_bytes,bpv,ratio_vs_paper,hard_cap_margin_B,ideal_cap_margin_B,decode_rc,counts_ok,connectivity_ok,topology_ok,nearest_geometry_ok,manifold_profile_ok,coordinate_remap_complete,cqst_raw_B,symbols_B,geometry_deltas_B,fixed_B,base_vertices,rms_max,mse_sym,note" \
> "${CSV}"

for r in ${ROUNDS_LIST}; do
    dir="${OUTDIR}/cq_r${r}"
    mkdir -p "${dir}"
    python3 - "${dir}" "${r}" "${MESH}" "${CONQUEST_CFG}" <<'PYEOF'
import json, sys
d, r, mesh, cfg_path = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
j = json.load(open(cfg_path))
j['input'] = mesh
j['output'] = d + '/art'
j['rounds'] = r
j['quantization'] = True
j['prediction'] = False
j['segmentation'] = False
j['adjustment'] = False
j['compression'] = True
j.setdefault('extra', {})
j['extra']['PMSF_V2_COMPACT'] = '1'
j['extra']['PMSF_SYMBOL_BLOB_ZLIB'] = '1'
j['extra']['PMSF_DEFAULT_QUANT_BITS'] = '12'
j['extra']['PMSF_POOL_V'] = '100000'
j['extra']['PMSF_POOL_H'] = '480000'
j['extra']['PMSF_POOL_F'] = '200000'
j['extra']['PMSF_REPAIR_DEGENERATE'] = '1'
json.dump(j, open(d + '/c.json', 'w'), indent=2)
json.dump({"algorithm": "conquest", "backend": "columnar",
           "input": d + '/art', "output": d + '/art.dec.off'},
          open(d + '/d.json', 'w'))
PYEOF

    envs=()
    [[ "${EB_NATIVE}" != "0" ]] && envs+=("PMSF_BASE_EDGEBREAKER_NATIVE=1")
    env "${envs[@]}" "${COMPRESS_BIN}" "${dir}/c.json" > "${dir}/c.log" 2>&1
    compress_rc=$?

    compact_b=0; parquet_b=0
    [[ -f "${dir}/art_compact.bin" ]] && compact_b=$(stat -c '%s' "${dir}/art_compact.bin")
    [[ -f "${dir}/art_symbols.parquet" ]] && parquet_b=$(stat -c '%s' "${dir}/art_symbols.parquet")
    bytes=$((compact_b + parquet_b))

    "${DECOMPRESS_BIN}" "${dir}/d.json" > "${dir}/d.log" 2>&1
    decode_rc=$?
    flags=",,,,,,"
    rms=""; mse=""
    if [[ ${decode_rc} -eq 0 && -f "${dir}/art.dec.off" ]]; then
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
    print(f"{d.get('rms_max', '')}", f"{d.get('mse_sym', '')}")
except Exception:
    print("", "")
PYEOF
)
    fi

    # CQST component split (connectivity symbols vs geometry deltas vs headers)
    split="$("${PYSPLIT}" - "${dir}/art_symbols.parquet" "${bytes}" <<'PYEOF'
import sys
try:
    import pyarrow.parquet as pq
except Exception:
    print(",,,,,")
    raise SystemExit(0)
import struct, ctypes, ctypes.util
path = sys.argv[1]
blob = pq.read_table(path).column('rc_blob').to_pylist()[0]
sbc_len = len(blob)
if blob[:4] == b'SBC1':
    comp_len = struct.unpack_from('<I', blob, 9)[0]
    raw_len = struct.unpack_from('<I', blob, 5)[0]
    try:
        lib = ctypes.CDLL(ctypes.util.find_library('zstd'))
        lib.ZSTD_decompress.restype = ctypes.c_size_t
        out = ctypes.create_string_buffer(raw_len)
        n = lib.ZSTD_decompress(out, raw_len, blob[13:13 + comp_len], comp_len)
        raw = out.raw[:n]
    except Exception:
        print(",,,,,"); raise SystemExit(0)
else:
    raw = blob
if raw[:4] != b'CQST':
    print(",,,,,"); raise SystemExit(0)
nlayers = struct.unpack_from('<I', raw, 40)[0]
off = 44
sym = delta = 0
removed = 0
for _ in range(nlayers):
    off += 48
    nsym, sb = struct.unpack_from('<II', raw, off); off += 8
    sym += sb; off += sb
    nd = struct.unpack_from('<I', raw, off)[0]; off += 4
    removed += nd
    for _k in range(3):
        lo, span, by = struct.unpack_from('<iII', raw, off); off += 12 + by; delta += by
print(f"{len(raw)},{sym},{delta},{len(raw)-sym-delta},{19851-removed}")
PYEOF
)"
    IFS=',' read -r cqst_raw sym_b delta_b fixed_b base_v <<< "${split}"

    bpv=""; ratio=""; margin=""; ideal_margin=""
    if [[ "${bytes}" -gt 0 ]]; then
        bpv="$(python3 -c "print(f'{8.0*${bytes}/${NV}:.6f}')")"
        ratio="$(python3 -c "print(f'{8.0*${bytes}/${NV}/17.0:.6f}')")"
        margin=$((CAP_BYTES - bytes))
        ideal_margin=$((IDEAL_BYTES - bytes))
    fi
    note=""
    if [[ -n "${bpv}" ]]; then
        python3 -c "import sys; sys.exit(0 if ${bpv} <= ${HARD_CAP_BPV} else 1)" \
            && note="PASS_hard_cap" || note="FAIL_hard_cap"
    fi
    printf '%s\n' "${r},${compact_b},${parquet_b},${bytes},${bpv},${ratio},${margin},${ideal_margin},${decode_rc},${flags},${cqst_raw},${sym_b},${delta_b},${fixed_b},${base_v},${rms},${mse},${note}" >> "${CSV}"
    echo "[p3-backbone] r=${r}: ${bytes} B bpv=${bpv} margin=${margin} B flags=${flags} ${note}"
done

echo "[p3-backbone] baseline MMSE canonical = ${BASELINE_BYTES} B / ${BASELINE_BPV} bpv"
echo "[p3-backbone] hard cap ${HARD_CAP_BPV} bpv = ${CAP_BYTES} B ; ideal ${IDEAL_CAP_BPV} bpv = ${IDEAL_BYTES} B"
echo "[p3-backbone] CSV: ${CSV}"
