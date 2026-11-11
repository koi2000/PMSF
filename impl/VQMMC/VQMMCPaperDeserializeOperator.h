// SPDX-License-Identifier: Apache-2.0
//
// VQMMCPaperDeserializeOperator.h — M5-5 (task-35) VQMMC paper decode entry.
//
// The paper artifact carries the canonical binary stream (header + quantized
// base mesh + single CONN layout-30 blob) verbatim as an opaque payload inside
// a P2BL v2 container. This deserializer only replaces `init(path)`: it reads
// the opaque payload, strips the legacy u32 content-length prefix and hands the
// remaining buffer to the unchanged `VQMMCDeserializeOperator` strict path
// (VQMMCStrictSymbolReadOperator + VQMMCReconstructOperator + VQMMCPostProcessor).
#ifndef PMSF_VQMMC_PAPER_DESERIALIZE_OPERATOR_H
#define PMSF_VQMMC_PAPER_DESERIALIZE_OPERATOR_H

#include "VQMMCDeserializeOperator.h"

#include <memory>
#include <string>

class VQMMCPaperDeserializeOperator : public VQMMCDeserializeOperator {
  public:
    VQMMCPaperDeserializeOperator(
        std::shared_ptr<VQMMCSymbolReadOperator> symbolReadOperator,
        std::shared_ptr<ReconstructOperator> reconstructOperator,
        std::shared_ptr<VQMMCPostProcessor> postProcessor)
        : VQMMCDeserializeOperator(symbolReadOperator, reconstructOperator,
                                   postProcessor) {}
    ~VQMMCPaperDeserializeOperator() = default;

    /// @brief Read the P2BL opaque-payload artifact and install the canonical
    ///        binary stream into the base buffer_, then force strict mode.
    void init(std::string path) override;
    void setQueryMaxLod(int maxLod) override { queryMaxLod_ = maxLod; }
    int queryMaxLod() const override { return queryMaxLod_; }
    bool supportsRandomAccess() const override { return true; }

  private:
    int queryMaxLod_ = -1;
};

#endif  // PMSF_VQMMC_PAPER_DESERIALIZE_OPERATOR_H
