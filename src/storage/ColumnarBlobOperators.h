#ifndef PMSF_STORAGE_COLUMNAR_BLOB_OPERATORS_H
#define PMSF_STORAGE_COLUMNAR_BLOB_OPERATORS_H

#include "compress/SerializeOperator.h"
#include "decompress/DeserializeOperator.h"
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {

enum class ColumnarBlobFormat {
    ArrowIpc,
    ORC,
};

inline constexpr std::int32_t kColumnarBlobWrapperCodecVersion = 1;

inline const char* columnarBlobFormatName(ColumnarBlobFormat format) {
    switch (format) {
        case ColumnarBlobFormat::ArrowIpc: return "arrow";
        case ColumnarBlobFormat::ORC: return "orc";
    }
    return "unknown";
}

inline std::string columnarPayloadName(ColumnarBlobFormat format) {
    return std::string(columnarBlobFormatName(format)) + "_columnar_payload";
}

inline std::string columnarPayloadCodec(ColumnarBlobFormat format) {
    return std::string(columnarBlobFormatName(format)) + ".columnar";
}

inline PMSFPayloadDescriptor columnarPayloadDescriptor(
    ColumnarBlobFormat format) {
    return makePMSFPayloadDescriptor(PMSFPayloadRole::ColumnarContainer,
                                      columnarPayloadName(format),
                                      columnarPayloadCodec(format));
}

inline PMSFSymbolPayloadSpec columnarPayloadSpec(ColumnarBlobFormat format) {
    return paperPayloadSpecFromDescriptor(columnarPayloadDescriptor(format));
}

inline void appendColumnarBlobPayload(PMSFSymbolPayloadTable& rows,
                                      ColumnarBlobFormat format,
                                      const void* data,
                                      std::size_t size) {
    appendPMSFPayloadRecordFromDescriptor(
        rows, columnarPayloadDescriptor(format), data, size);
}

inline std::string columnarSidecarPrefix(ColumnarBlobFormat format) {
    return std::string(columnarBlobFormatName(format)) + "_columnar";
}

inline bool columnarSkipCompatBlobFileRequested() {
    return pmsfStorageEnvEnabled("PMSF_COLUMNAR_BLOB_SKIP_COMPAT_FILE");
}

class ColumnarBlobSerializeOperator : public SerializeOperator {
  public:
    ColumnarBlobSerializeOperator(std::shared_ptr<SerializeOperator> inner,
                                  ColumnarBlobFormat format);
    ~ColumnarBlobSerializeOperator() override = default;

    void init() override;
    void serializeInt(int val) override;
    void serializeHeader(PMSFHeader header) override;
    void setHeader(std::shared_ptr<PMSFHeader> header) override;
    void serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) override;
    void serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh,
                                   std::vector<MCGAL::Halfedge*> seeds) override;
    void serialize(std::string path) override;

  private:
    std::shared_ptr<SerializeOperator> inner_;
    ColumnarBlobFormat format_;
};

class ColumnarBlobDeserializeOperator : public DeserializeOperator {
  public:
    ColumnarBlobDeserializeOperator(std::shared_ptr<DeserializeOperator> inner,
                                    ColumnarBlobFormat format);
    ~ColumnarBlobDeserializeOperator() = default;

    void init(std::string path) override;
    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override;
    void deserializeHeader(PMSFHeader& header) override;
    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;
    void deserializeOneRound() override;
    std::shared_ptr<MCGAL::Mesh> exportMesh() override;
    PMSFHeader header() override;

  private:
    void syncFromInner();

    std::shared_ptr<DeserializeOperator> inner_;
    ColumnarBlobFormat format_;
};

class ArrowBlobSerializeOperator : public ColumnarBlobSerializeOperator {
  public:
    explicit ArrowBlobSerializeOperator(std::shared_ptr<SerializeOperator> inner)
        : ColumnarBlobSerializeOperator(std::move(inner), ColumnarBlobFormat::ArrowIpc) {}
};

class ArrowBlobDeserializeOperator : public ColumnarBlobDeserializeOperator {
  public:
    explicit ArrowBlobDeserializeOperator(std::shared_ptr<DeserializeOperator> inner)
        : ColumnarBlobDeserializeOperator(std::move(inner), ColumnarBlobFormat::ArrowIpc) {}
};

class ORCBlobSerializeOperator : public ColumnarBlobSerializeOperator {
  public:
    explicit ORCBlobSerializeOperator(std::shared_ptr<SerializeOperator> inner)
        : ColumnarBlobSerializeOperator(std::move(inner), ColumnarBlobFormat::ORC) {}
};

class ORCBlobDeserializeOperator : public ColumnarBlobDeserializeOperator {
  public:
    explicit ORCBlobDeserializeOperator(std::shared_ptr<DeserializeOperator> inner)
        : ColumnarBlobDeserializeOperator(std::move(inner), ColumnarBlobFormat::ORC) {}
};

}  // namespace pmsf

#endif
