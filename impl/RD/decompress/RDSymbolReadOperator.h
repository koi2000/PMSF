/*
 * @file RDSymbolReadOperator.h
 * @brief RD paper-aligned gate-conquest reader（Phase A）。
 *
 * 与 RDSymbolCollectOperator 对称：Phase A 复用 ValenceSymbolReadOperator 的
 * BFS + RC 入口。通过在 `ColumnarDeserializeOperator` 里把 `BlobLayout::RD_GATE=7`
 * dispatch 到本类实例化即可把 RD blob 和 Valence blob 在解码侧分离。
 *
 * Phase B 打算实现：真·gate-only BFS、Frenet 逆旋转、v=3 cleaning 层差异化模型。
 * 为避免 Phase A 引入 round-trip 风险，这些改动全部延后到 RDSymbolReadOperator
 * 自身而不污染 Valence path。
 */
#pragma once

#include "decompress/ValenceSymbolReadOperator.h"

class RDSymbolReadOperator : public ValenceSymbolReadOperator {
  public:
    RDSymbolReadOperator() = default;

    // Phase A：不覆写任何方法。Parquet V2 deserialize 通过 blob_layout==7 分派到
    // 本实例，行为完全等价于 Valence。未来 Phase B 在此覆写
    // `importFromRangeCodedBuffer` 与 `collectFromDecoded` 即可。
};
