/*
 * @file PaperSerializeOperator.h
 * @brief PaperStream serializer adapter (M2b pilot).
 *
 * Strategy for the pilot: reuse ColumnarSerializeOperator to produce the
 * legacy compact bin/base mesh (which already contains the header + base mesh),
 * then use PaperStreamSymbolAdapter to turn the collector's PMSFSymbolTable
 * into PaperStream rows.  The result is a single <base>_paper.parquet.
 *
 * This is deliberately a bridge: the final M2c path will serialize the base
 * mesh directly into the paper BLOB instead of calling the legacy serializer.
 */
#ifndef PMSF_COMPRESS_PAPER_SERIALIZE_OPERATOR_H
#define PMSF_COMPRESS_PAPER_SERIALIZE_OPERATOR_H

#include "compress/ColumnarSerializeOperator.h"

#include <string>

class PaperSerializeOperator : public ColumnarSerializeOperator {
  public:
    PaperSerializeOperator(std::shared_ptr<SymbolCollectOperator> collector,
                           bool enablePrediction,
                           bool enableQuantization,
                           bool enableCompress,
                           std::string algorithmName)
        : ColumnarSerializeOperator(collector, enablePrediction,
                                    enableQuantization, enableCompress),
          algorithmName_(std::move(algorithmName)) {}

    void serialize(std::string path) override;

  private:
    std::string algorithmName_;
};

#endif  // PMSF_COMPRESS_PAPER_SERIALIZE_OPERATOR_H
