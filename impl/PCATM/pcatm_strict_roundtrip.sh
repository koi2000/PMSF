#!/usr/bin/env bash
# PCATM strict-paper faithful round trip (WS-B task-5).
# Best faithful point: horse19k q12 rounds=40 ->
#   71,550 B = 28.834819 bpv (1.365x paper 21.14), decode all true.
# Requires the two fixes in impl/common/strict + impl/PCATM (see PCATM_STRICT_FIX.md).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="${3:-${ROOT}/build}"
INPUT="${1:-${ROOT}/datasets/PaperModels/horse19k.off}"
OUTDIR="${2:-/tmp/pcatm_strict_roundtrip}"
ROUNDS="${PCATM_STRICT_ROUNDS:-40}"
rm -rf "$OUTDIR"; mkdir -p "$OUTDIR"
python3 - "$INPUT" "$OUTDIR/art" "$ROUNDS" "$OUTDIR/c.json" <<'PY'
import json,sys
inp,out,rounds,cfg=sys.argv[1:5]
extra={"PMSF_V2_COMPACT":"1","PMSF_DEFAULT_QUANT_BITS":"12","PMSF_PCATM_QUANT_BITS":"12",
       "PMSF_PCATM_USE_STRICT_PAPER":"1","PMSF_PCATM_STRICT_FAN_FALLBACK":"1",
       "PMSF_REPAIR_DEGENERATE":"1","PMSF_BASE_EDGEBREAKER_NATIVE":"1",
       "PMSF_PARQUET_ZSTD_LEVEL":"22","PMSF_POOL_V":"200000","PMSF_POOL_H":"1000000",
       "PMSF_POOL_F":"400000"}
json.dump({"algorithm":"PCATM","backend":"columnar","input":inp,"output":out,
           "rounds":int(rounds),"quantization":True,"prediction":True,
           "segmentation":False,"adjustment":True,"compression":True,
           "extra":extra},open(cfg,"w"),indent=1)
PY
export PMSF_V2_COMPACT=1 PMSF_DEFAULT_QUANT_BITS=12 PMSF_PCATM_QUANT_BITS=12
export PMSF_PCATM_USE_STRICT_PAPER=1 PMSF_PCATM_STRICT_FAN_FALLBACK=1
export PMSF_REPAIR_DEGENERATE=1 PMSF_BASE_EDGEBREAKER_NATIVE=1 PMSF_PARQUET_ZSTD_LEVEL=22
export PMSF_POOL_V=200000 PMSF_POOL_H=1000000 PMSF_POOL_F=400000 PMSF_BPV_DIAG=1
"$BUILD/PMSFCompress" "$OUTDIR/c.json" >"$OUTDIR/compress.log" 2>&1
python3 - "$OUTDIR/art" "$OUTDIR/dec.off" "$OUTDIR/d.json" <<'PY'
import json,sys
art,dec,cfg=sys.argv[1:4]
json.dump({"algorithm":"PCATM","backend":"columnar","input":art,"output":dec},open(cfg,"w"),indent=1)
PY
"$BUILD/PMSFDecompress" "$OUTDIR/d.json" >"$OUTDIR/decompress.log" 2>&1
"$BUILD/MeshCompare" "$INPUT" "$OUTDIR/dec.off" --tol 1e-3 >"$OUTDIR/meshcompare.log" 2>&1 || true
if [[ -x "$BUILD/SurfaceMetrics" ]]; then
  "$BUILD/SurfaceMetrics" "$INPUT" "$OUTDIR/dec.off" --json --samples 5000 >"$OUTDIR/surface_metrics.json" 2>/dev/null || true
fi
V=$(awk 'BEGIN{s=0} {gsub(/\r/,"")} /^#/{next} s==0 && $1=="OFF"{s=1; if(NF>=4){print $2; exit}; next} s==0{print $1; exit} s==1{print $1; exit}' "$INPUT")
BYTES=$(find "$OUTDIR" -maxdepth 1 -type f \( -name 'art*.bin' -o -name 'art*.parquet' \) -printf '%s\n' | awk '{s+=$1} END{print s+0}')
awk -v b="$BYTES" -v v="$V" 'BEGIN{printf "PCATM strict: bytes=%d bpv=%.6f paper=21.14 ratio=%.4f\n", b, b*8/v, (b*8/v)/21.14}'
grep -F '[PCATMStrictPaper bpv' "$OUTDIR/compress.log" | tail -1 || true
grep -E '^(counts_ok|connectivity_ok|topology_ok|nearest_geometry_ok|coordinate_remap_complete|manifold_profile_ok)=' "$OUTDIR/meshcompare.log" || true
grep -E '"(rms_max|mse_sym)"' "$OUTDIR/surface_metrics.json" 2>/dev/null || true
