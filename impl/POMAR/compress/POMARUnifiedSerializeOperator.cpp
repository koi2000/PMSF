#include "POMAR/compress/POMARUnifiedSerializeOperator.h"
#include "compress/UnifiedSerializeOperator.h"
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "storage/PMSFUnifiedArtifactWriter.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

void POMARUnifiedSerializeOperator::serialize(std::string path) {
    // 1. 让 V2 专属 writer 写 4 张 `<base>_v3_*.parquet`（V2 reader 可读）。
    //    **阻止**父类释放 buffer_ —— 我们稍后还要读 rawBuffer() 写 blob 表。
    setKeepBufferAfterSerialize(true);
    POMARColumnarSerializeOperator::serialize(
        pmsf::unifiedCompatOutputPath(path));

    // 2. 额外写 `<base>_v3_blob.parquet` —— 把 4 段原样记录为统一 paper payload，
    //    方便 V3 通用 reader 一次性拿到完整 inner buffer。codec 用 "pomar.v3"。
    const char*       buf     = rawBuffer();
    const std::size_t bufSize = rawBufferSize();
    const std::size_t baseEnd = rawBaseMeshEnd();
    const std::size_t c0End   = rawCluster0End();
    const std::size_t metaEnd = rawClusterMetaEnd();

    if (bufSize == 0 || baseEnd > c0End || c0End > metaEnd || metaEnd > bufSize) {
        std::cerr << "[POMAR-PV3] blob table skipped: inner buffer offsets inconsistent"
                  << std::endl;
        releaseBufferNow();
        return;
    }

    std::string base = pmsf::unifiedStemFromArtifactPath(path);

    auto buildPayloadBlobs = [&]() {
        pmsf::PMSFSymbolPayloadTable payloadRows =
            pmsf::buildPomarPayloadTable(
                pmsf::kPomarV3PayloadCodecName,
                buf,
                baseEnd,
                c0End,
                metaEnd,
                bufSize);
        return pmsf::paperNamedBlobsFromPayloadRows(payloadRows);
    };

    if (pmsf::unifiedStorageEnabled()) {
        std::vector<pmsf::PaperNamedBlob> namedBlobs = buildPayloadBlobs();
        writeUnifiedPMSFArtifactsForSerializerAndNamedBlobs(
            base, std::move(namedBlobs), false);
    } else {
        std::vector<pmsf::PaperNamedBlob> namedBlobs = buildPayloadBlobs();
        const std::string blobPath = pmsf::paperPayloadBlobOutputPathForBase(base);
        if (!pmsf::writePaperNamedBlobsForBase(base, namedBlobs)) {
            throw std::runtime_error("POMARv3: failed to write unified blob table: " + blobPath);
        }
        std::cout << "[POMAR-PV3] blob table: " << blobPath
                  << " entries=" << namedBlobs.size() << std::endl;
    }

    // 3. 现在可以安全释放父类 buffer。
    releaseBufferNow();
}
