/*
 * @file PaperDeserializeOperator.h
 * @brief PaperStream deserializer adapter (M2b pilot).
 *
 * The pilot stores the legacy compact bin (header + base mesh) in the paper
 * partition BLOB and the symbol table in paper LOD rows.  This operator
 * reconstructs a temporary legacy compact bin, imports the paper symbol table
 * into a ColumnarDeserializeOperator, and delegates the round state machine to
 * it.  M2c replaces the bridge with a native paper decoder.
 */
#ifndef PMSF_DECOMPRESS_PAPER_DESERIALIZE_OPERATOR_H
#define PMSF_DECOMPRESS_PAPER_DESERIALIZE_OPERATOR_H

#include "decompress/ColumnarDeserializeOperator.h"
#include "decompress/DeserializeOperator.h"
#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <memory>
#include <string>

namespace pmsf { class PaperStreamReaderV2; }

class PaperDeserializeOperator : public DeserializeOperator {
  public:
    PaperDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                             std::shared_ptr<ReconstructOperator> reconstructOperator);
    ~PaperDeserializeOperator();

    void init(std::string path) override;
    void deserializeHeader(PMSFHeader& header) override;
    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;
    std::shared_ptr<MCGAL::Mesh> exportMesh() override;
    PMSFHeader header() override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override;
    void setQueryRegion(int regionId) override;
    void setQueryMaxLod(int maxLod) override;
    int queryMaxLod() const override;
    bool supportsRandomAccess() const override;
    void originalMeshCounts(int& v, int& f) const override;
    void clearPMSFSymbolContext() override;

    // ---- M4-3 v2 bridge dispatch (only active for version-2 artifacts) ----
    bool usedV2Path() const { return usedV2Path_; }
    std::int32_t v2LodCount() const { return v2LodCount_; }
    const pmsf::PaperStreamTable& v2Table() const { return v2Table_; }

  private:
    void initV2(pmsf::PaperStreamReaderV2& reader);

    std::unique_ptr<ColumnarDeserializeOperator> inner_;
    std::string tempDir_;
    bool usedV2Path_ = false;
    bool v2Native_ = false;
    bool v2Typed_ = false;
    std::int32_t v2LodCount_ = 0;
    std::int32_t v2OrigV_ = 0;
    std::int32_t v2OrigF_ = 0;
    int queryMaxLod_ = -1;
    pmsf::PaperStreamTable v2Table_;
};

#endif  // PMSF_DECOMPRESS_PAPER_DESERIALIZE_OPERATOR_H
