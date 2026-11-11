#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
#
# impl/MMSE/mmse_p1_predictor_ablation.sh
#
# P1 (task-15, WS-B4, 2026-10-22): MMSE paper predictor-stack A/B evidence.
#
# Runs the canonical MMSE row (horse19k, q12, rounds=50, columnar backend) under
# several `PMSF_MMSE_PREDICTOR` settings and records, per row:
#   * artifact total bytes + bpv (8*bytes / |V_orig|),
#   * per-stream bytes from `PMSF_BPV_DIAG` (valence / isPoly / residual / container),
#   * decoder rc + MeshCompare flags (counts/connectivity/topology/nearest/...),
#   * sha256 of the decoded OFF (must be identical to the baseline row: the
#     predictor only changes the *residual representation*, so a symmetric
#     encoder/decoder pair must reconstruct exactly the same quantized mesh),
#   * the encoded-mode histogram (from the `[MMSE-DR predictor]` diagnostic).
#
# Faithful-path rules (Lead 2026-10-22): q12, horse19k, no q/variant tricks.
# The predictor stack only changes geometry prediction; connectivity (valence +
# isPoly) is untouched.  Rows are *ablation*, not gate variants.
#
# Usage:
#   impl/MMSE/mmse_p1_predictor_ablation.sh [out_dir]
#
# Env:
#   BUILD_DIR     build tree (default <root>/build)
#   MESH          input OFF (default datasets/PaperModels/horse19k.off)
#   ROUNDS        rounds (default 50)
#   MMSE_P1_CONFIGS  space separated "name:env..." rows (default table below)

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
COMPRESS_BIN="${BUILD_DIR}/PMSFCompress"
DECOMPRESS_BIN="${BUILD_DIR}/PMSFDecompress"
COMPARE_BIN="${BUILD_DIR}/MeshCompare"
MESH="${MESH:-${ROOT_DIR}/datasets/PaperModels/horse19k.off}"
ROUNDS="${ROUNDS:-50}"
OUTDIR="${1:-${ROOT_DIR}/build/phase1/mmse_p1}"
CSV="${OUTDIR}/mmse_p1_predictor_ablation.csv"

for bin in "${COMPRESS_BIN}" "${DECOMPRESS_BIN}" "${COMPARE_BIN}"; do
    [[ -x "${bin}" ]] || { echo "missing binary ${bin}; run scripts/tests/build.sh" >&2; exit 3; }
done
[[ -f "${MESH}" ]] || { echo "missing mesh ${MESH}" >&2; exit 3; }

mkdir -p "${OUTDIR}"
OUTDIR="$(cd "${OUTDIR}" && pwd)"

# name | PMSF_MMSE_PREDICTOR | PMSF_MMSE_CLUSTER | PMSF_MMSE_LS
CONFIGS=(
    "baseline|||"
    "bary:mode0|bary||"
    "dr05:fixed|dr05||"
    "dr1:fixed|dr1||"
    "ls:fixed|ls||1"
    "mode:decision|mode|32|1"
    "mode:cluster8|mode|8|1"
    "mode:cluster16|mode|16|1"
    "mode:cluster64|mode|64|1"
    "mode:cluster128|mode|128|1"
    "mode:noLS|mode|32|0"
)

VERTICES="$(python3 - "${MESH}" <<'PYEOF'
import sys
path = sys.argv[1]
with open(path, errors="replace") as fh:
    header = fh.readline().strip()
    if header.startswith("OFF"):
        rest = header[3:].split()
        if len(rest) >= 2:
            print(int(rest[0])); sys.exit(0)
        print(int(fh.readline().split()[0])); sys.exit(0)
print(0)
PYEOF
)"
[[ "${VERTICES}" =~ ^[0-9]+$ && "${VERTICES}" -gt 0 ]] || { echo "cannot parse ${MESH}" >&2; exit 3; }

printf '%s\n' \
"name,predictor,cluster,ls_enabled,artifact_bytes,bpv,compress_rc,decode_rc,counts_ok,connectivity_ok,topology_ok,nearest_geometry_ok,manifold_profile_ok,coordinate_remap_complete,valence_B,isPolyBit_B,residual_B,container_B,blast_total_B,mode_header_B,residual_RC_B,pred_gain_B,dec_diff_verts,dec_diff_max,dec_sha256,dec_off_bytes,note" \
> "${CSV}"

BASE_SHA=""
BASE_RES_RC=""
BASE_DEC=""
row=0
for cfg in "${CONFIGS[@]}"; do
    IFS='|' read -r name pred cluster ls <<< "${cfg}"
    dir="${OUTDIR}/${name//:/_}"
    mkdir -p "${dir}"
    prefix="${dir}/art"

    # ---- config JSON (MMSE faithful row + optional predictor env) ----
    PRED_ENV="${pred}" CLUSTER_ENV="${cluster}" LS_ENV="${ls}" \
    MESH_PATH="${MESH}" PREFIX="${prefix}" ROUNDS="${ROUNDS}" \
    python3 - <<'PYEOF'
import json, os
extra = {
    "PMSF_V2_COMPACT": "1",
    "PMSF_REPAIR_DEGENERATE": "1",
    "PMSF_DEFAULT_QUANT_BITS": "12",
    "PMSF_MMSE_QUANT_BITS": "12",
    "PMSF_MMSE_USE_DUAL_RING": "1",
    "PMSF_POOL_V": "100000",
    "PMSF_POOL_H": "480000",
    "PMSF_POOL_F": "200000",
}
if os.environ.get("PRED_ENV"):
    extra["PMSF_MMSE_PREDICTOR"] = os.environ["PRED_ENV"]
if os.environ.get("CLUSTER_ENV"):
    extra["PMSF_MMSE_CLUSTER"] = os.environ["CLUSTER_ENV"]
if os.environ.get("LS_ENV"):
    extra["PMSF_MMSE_LS"] = os.environ["LS_ENV"]
base = {
    "algorithm": "MMSE", "backend": "columnar", "rounds": int(os.environ["ROUNDS"]),
    "quantization": True, "prediction": True, "segmentation": False,
    "adjustment": True, "compression": True,
    "input": os.environ["MESH_PATH"], "output": os.environ["PREFIX"] + ".parquet",
    "extra": extra,
}
json.dump(base, open(os.environ["PREFIX"] + ".c.json", "w"), indent=2)
json.dump({"algorithm": "MMSE", "backend": "columnar",
           "input": os.environ["PREFIX"], "output": os.environ["PREFIX"] + ".dec.off"},
          open(os.environ["PREFIX"] + ".d.json", "w"), indent=2)
PYEOF

    rm -f "${prefix}"_* "${prefix}".dec.off
    PMSF_BPV_DIAG=1 "${COMPRESS_BIN}" "${prefix}.c.json" > "${dir}/c.log" 2>&1
    compress_rc=$?

    bytes=0
    shopt -s nullglob
    for f in "${prefix}"_*.parquet "${prefix}"_*.bin; do
        [[ -f "${f}" ]] && bytes=$((bytes + $(stat -c '%s' "${f}")))
    done
    shopt -u nullglob
    bpv="$(python3 -c "print(f'{8.0*${bytes}/${VERTICES}:.6f}')")"

    val="$(grep -oP 'valence=\K[0-9]+' "${dir}/c.log" | head -1)"
    isp="$(grep -oP 'isPolyBit=\K[0-9]+' "${dir}/c.log" | head -1)"
    res="$(grep -oP 'residual=\K[0-9]+' "${dir}/c.log" | head -1)"
    con="$(grep -oP 'containerOverhead=\K[0-9]+' "${dir}/c.log" | head -1)"
    blast="$(grep -oP 'totalBlob=\K[0-9]+' "${dir}/c.log" | head -1)"
    modehist="$(grep -oP 'modeHist\(bary,dr05,dr1,ls\)=\K[0-9,]+' "${dir}/c.log" | head -1)"
    note="${modehist:+modeHist=${modehist}}"

    "${DECOMPRESS_BIN}" "${prefix}.d.json" > "${dir}/d.log" 2>&1
    decode_rc=$?
    flags=""
    if [[ ${decode_rc} -eq 0 && -f "${prefix}.dec.off" ]]; then
        "${COMPARE_BIN}" "${MESH}" "${prefix}.dec.off" --tol 0.001 > "${dir}/compare.log" 2>&1
        flags="$(python3 - "${dir}/compare.log" <<'PYEOF'
import sys
want = ["counts_ok","connectivity_ok","topology_ok","nearest_geometry_ok",
        "manifold_profile_ok","coordinate_remap_complete"]
vals = {k: "" for k in want}
for line in open(sys.argv[1], errors="replace"):
    if "=" in line:
        k, v = line.strip().split("=", 1)
        k, v = k.strip(), v.strip()
        if k in vals:
            vals[k] = v
print(",".join(vals[k] for k in want))
PYEOF
)"
    else
        flags=",,,,,"
    fi
    dec_sha=""; dec_bytes=""
    if [[ -f "${prefix}.dec.off" ]]; then
        dec_sha="$(sha256sum "${prefix}.dec.off" | cut -d' ' -f1)"
        dec_bytes="$(stat -c '%s' "${prefix}.dec.off")"
        if [[ -z "${BASE_SHA}" ]]; then
            BASE_SHA="${dec_sha}"
            BASE_DEC="${prefix}.dec.off"
        fi
    fi
    # mode-stream header bytes (inside the residual stream): fixed = 1 B/round,
    # per-cluster = 3 B/round + ceil(clusters/4).  Derived from the encoder diag.
    mode_hdr="$(python3 - "${dir}/c.log" <<'PYEOF2'
import re, sys
total = 0
for line in open(sys.argv[1], errors="replace"):
    m = re.search(r"\[MMSE-DR pred\] modes=(\d+) clusterSize=(\d+)", line)
    if not m:
        continue
    nmodes = int(m.group(1))
    total += 1 if nmodes == 1 else 3 + (nmodes + 3) // 4
print(total)
PYEOF2
)"
    [[ "${mode_hdr}" =~ ^[0-9]+$ ]] || mode_hdr=0
    res_num="${res:-0}"; [[ "${res_num}" =~ ^[0-9]+$ ]] || res_num=0
    res_rc=$((res_num - mode_hdr))
    if [[ -n "${BASE_RES_RC}" ]]; then pred_gain=$((BASE_RES_RC - res_rc)); else pred_gain=0; fi
    [[ -z "${BASE_RES_RC}" && -n "${val}" ]] && BASE_RES_RC="${res_rc}"
    # decoded-mesh difference vs the baseline row (symmetric-predictor check)
    dec_diff_verts=""; dec_diff_max=""
    if [[ -f "${prefix}.dec.off" && -n "${BASE_DEC}" && -f "${BASE_DEC}" ]]; then
        read -r dec_diff_verts dec_diff_max < <(python3 - "${BASE_DEC}" "${prefix}.dec.off" <<'PYEOF3'
import sys
def load_off(p):
    toks = []
    for line in open(p, errors="replace"):
        if line.startswith("#"):
            continue
        toks.extend(line.split())
    nv = int(toks[1])
    vs = []
    i = 3
    for _ in range(nv):
        vs.append((float(toks[i]), float(toks[i + 1]), float(toks[i + 2])))
        i += 3
    return vs
a = load_off(sys.argv[1]); b = load_off(sys.argv[2])
nv = min(len(a), len(b))
nd = 0
md = 0.0
for i in range(nv):
    d = max(abs(x - y) for x, y in zip(a[i], b[i]))
    if d > 0:
        nd += 1
        md = max(md, d)
nd += abs(len(a) - len(b))
print(nd, f"{md:.3e}")
PYEOF3
)
    fi
    printf '%s\n' "${name},${pred},${cluster},${ls},${bytes},${bpv},${compress_rc},${decode_rc},${flags},${val},${isp},${res},${con},${blast},${mode_hdr},${res_rc},${pred_gain},${dec_diff_verts},${dec_diff_max},${dec_sha},${dec_bytes},${note}" >> "${CSV}"
    row=$((row + 1))
    echo "[mmse-p1] ${name}: ${bytes} B ${bpv} bpv pred=${pred:-off} -> ${flags}"
done

echo "[mmse-p1] CSV: ${CSV}"
if [[ -n "${BASE_SHA}" ]]; then
    echo "[mmse-p1] baseline decoded sha256 = ${BASE_SHA}"
fi
