#!/usr/bin/env bash
# M5-4 PCLTTM PaperStream v2 round trip (task-34).
#
# Reproduces:
#   canonical PCLTTM r44/q12  = 66,755 B (832 compact + 65,923 residual parquet;
#                                payload = residual_data CONN 65,277 B)
#   paper-v2 fused            = 65,801 B (payload codec zstd) or 65,882 B (sbc1)
#   decoded paper file sha256 == canonical decoded sha256 (bit-identical)
#
# Root-cause note: paper encode/decode MUST set PMSF_PAPER_RETRI=1. Without it the
# paper backend stays on the generic bridge (layout 1, base 957 B) and the decoded
# artifact is not canonical-faithful (and decode without it segfaults).
#
# Usage: impl/PCLTTM/m5_4_paper_roundtrip.sh [out_dir] [payload_codec:zstd|sbc1]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${1:-/tmp/m5_4_pclttm_paper}"
CODEC="${2:-zstd}"
BUILD="${ROOT}/build"
INPUT="${ROOT}/datasets/PaperModels/horse19k.off"
ROUNDS=44
Q=12

[[ -x "$BUILD/PMSFCompress" && -x "$BUILD/PMSFDecompress" ]] || { echo "[err] build first" >&2; exit 1; }
rm -rf "$OUT"; mkdir -p "$OUT/canonical" "$OUT/paper"

export PMSF_V2_COMPACT=1
export PMSF_DEFAULT_QUANT_BITS="$Q" PMSF_PCLTTM_QUANT_BITS="$Q"
export PMSF_REPAIR_DEGENERATE=1
export PMSF_POOL_V=100000 PMSF_POOL_H=480000 PMSF_POOL_F=200000
export PMSF_BASE_EDGEBREAKER_NATIVE=1
export PMSF_PARQUET_ZSTD_LEVEL=22

python3 - "$INPUT" "$OUT" "$ROUNDS" "$Q" "$CODEC" <<'PYEOF'
import json, sys
inp, out, rounds, q, codec = sys.argv[1:6]
base_extra = {
    "PMSF_V2_COMPACT": "1", "PMSF_DEFAULT_QUANT_BITS": q, "PMSF_PCLTTM_QUANT_BITS": q,
    "PMSF_REPAIR_DEGENERATE": "1", "PMSF_POOL_V": "100000", "PMSF_POOL_H": "480000",
    "PMSF_POOL_F": "200000", "PMSF_BASE_EDGEBREAKER_NATIVE": "1",
    "PMSF_PARQUET_ZSTD_LEVEL": "22",
}
canon = {"algorithm": "PCLTTM", "backend": "columnar", "input": inp,
         "output": out + "/canonical/art", "rounds": int(rounds), "quantization": True,
         "prediction": True, "segmentation": False, "adjustment": True,
         "compression": True, "extra": base_extra}
json.dump(canon, open(out + "/canonical.json", "w"), indent=2)
paper_extra = dict(base_extra)
paper_extra.update({
    "PMSF_PAPERSTREAM_V2": "1", "PMSF_PAPERSTREAM_V2_PAYLOAD": "native",
    "PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC": codec,
    "PMSF_PAPERSTREAM_V2_CONTAINER": "blob",
    "PMSF_PAPERSTREAM_V2_BASE_SOURCE": "canonical",
})
paper = dict(canon); paper.update({"backend": "paper",
    "output": out + "/paper/art", "extra": paper_extra})
json.dump(paper, open(out + "/paper.json", "w"), indent=2)
dec = dict(canon); dec.update({"backend": "columnar", "input": out + "/canonical/art",
    "output": out + "/canonical/dec.off"})
json.dump(dec, open(out + "/canonical_dec.json", "w"), indent=2)
pdec = dict(paper); pdec.update({"input": out + "/paper/art",
    "output": out + "/paper/dec.off",
    "extra": dict(paper_extra, **{"PMSF_PAPER_RETRI": "1"})})
json.dump(pdec, open(out + "/paper_dec.json", "w"), indent=2)
PYEOF

"$BUILD/PMSFCompress" "$OUT/canonical.json"  > "$OUT/canonical_c.log" 2>&1
"$BUILD/PMSFDecompress" "$OUT/canonical_dec.json" > "$OUT/canonical_d.log" 2>&1
CANON_BYTES=$(( $(stat -c%s "$OUT/canonical/art_compact.bin") + $(stat -c%s "$OUT/canonical/art_residual.parquet") ))
CANON_SHA=$(sha256sum "$OUT/canonical/dec.off" | cut -d' ' -f1)

PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native \
PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC="$CODEC" PMSF_PAPERSTREAM_V2_CONTAINER=blob \
PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical PMSF_PAPER_RETRI=1 \
"$BUILD/PMSFCompress" "$OUT/paper.json" > "$OUT/paper_c.log" 2>&1

PMSF_PAPERSTREAM_V2=1 PMSF_PAPER_RETRI=1 \
"$BUILD/PMSFDecompress" "$OUT/paper_dec.json" > "$OUT/paper_d.log" 2>&1
PAPER_BYTES=$(stat -c%s "$OUT/paper/art_paper.parquet")
PAPER_SHA=$(sha256sum "$OUT/paper/dec.off" | cut -d' ' -f1)

BIT_IDENTICAL=$([[ "$CANON_SHA" == "$PAPER_SHA" ]] && echo true || echo false)
PASS=$([[ "$PAPER_BYTES" -le "$CANON_BYTES" && "$BIT_IDENTICAL" == true ]] && echo PASS || echo FAIL)

echo "canonical_bytes=$CANON_BYTES canonical_sha=$CANON_SHA"
echo "paper_bytes=$PAPER_BYTES paper_sha=$PAPER_SHA codec=$CODEC"
echo "bit_identical=$BIT_IDENTICAL pass=$PASS"
echo "run_dir=$OUT"

printf 'config,payload_codec,canonical_bytes,paper_bytes,delta_bytes,bpv_fused,canonical_decoded_sha,paper_decoded_sha,bit_identical,pass,run_dir\n' > "$OUT/evidence.csv"
python3 - "$CANON_BYTES" "$PAPER_BYTES" "$CANON_SHA" "$PAPER_SHA" "$BIT_IDENTICAL" "$PASS" "$OUT" "$CODEC" "$ROUNDS" <<'PYEOF' >> "$OUT/evidence.csv"
import sys
cb, pb, cs, ps, bi, p, out, codec, rounds = sys.argv[1:10]
print(f"pclttm_horse19k_q12_r{rounds},paper_v2_{codec},{cb},{pb},{int(pb)-int(cb)},{int(pb)*8/19851:.6f},{cs},{ps},{bi},{p},{out}")
PYEOF
cat "$OUT/evidence.csv"
