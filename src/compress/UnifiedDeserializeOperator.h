/*
 * @file UnifiedDeserializeOperator.h
 * @brief Minimal Parquet V3 single-file reader declaration.
 */
#ifndef PMSF_COMPRESS_UNIFIED_DESERIALIZE_OPERATOR_H
#define PMSF_COMPRESS_UNIFIED_DESERIALIZE_OPERATOR_H

#include "core/Symbol.h"

#include <string>
#include <vector>

namespace pmsf {

class UnifiedDeserializeOperator {
public:
    /// Read the full stream back from `path`, recovering the side-channel
    /// schema from the file's KV metadata.
    SymbolStream deserialize(const std::string& path);

    const std::vector<SideChannelSpec>& sideChannels() const { return sideChannels_; }
    bool hasPMSFSymbolTableMetadata() const {
        return pmsfSymbolTableMetadataPresent_;
    }
    bool isPMSFSymbolTable() const { return pmsfSymbolTableMode_; }
    bool hasPMSFSymbolMappingMetadata() const {
        return paperSymbolMappingMetadataPresent_;
    }
    bool hasPMSFSymbolMapping() const { return pmsfSymbolMappingMode_; }

private:
    std::vector<SideChannelSpec> sideChannels_;
    bool pmsfSymbolTableMetadataPresent_ = false;
    bool pmsfSymbolTableMode_ = false;
    bool paperSymbolMappingMetadataPresent_ = false;
    bool pmsfSymbolMappingMode_ = false;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_UNIFIED_DESERIALIZE_OPERATOR_H
