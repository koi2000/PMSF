/*
 * @file PaperStreamPageRAV1.h
 * @brief M6 (task-53) opt-in page-level random-access artifact profile (PRA1).
 *
 * Profile is additive: the official 7 fused `_paper.parquet` artifacts are NOT
 * modified.  A PRA1 file is a separate `_ra.parquet` carrying the same payload
 * split into independent chunks, one parquet row group per chunk, so
 * `readPrefix(k)` reads only the header row group + the first k chunk row
 * groups (bytesRead from parquet RG metadata, footer excluded).  The container
 * is a single BINARY column:
 *   row 0 (RG 0)      : PRA1 header row (format metadata + native header + base)
 *   row r (RG r)      : chunk r-1, [u32 rawLen][zlib stream]
 *
 * Kinds: 1 = QEM PSM2 per-round chunks, 2 = CONN LOD-block payload chunks.
 * This header is a standalone spike/helper profile; it does not touch the
 * official writer/reader dispatch.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_PAGE_RA_V1_H
#define PMSF_STORAGE_PAPER_STREAM_PAGE_RA_V1_H

#include <arrow/api.h>
#include <arrow/io/file.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>
#include <zlib.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pmsf {
namespace page_ra {

inline constexpr char kRaMagic[4] = {'P', 'R', 'A', '1'};
inline constexpr std::uint8_t kKindQem = 1;
inline constexpr std::uint8_t kKindConn = 2;
inline constexpr std::uint8_t kChunkCodecZlib = 1;

struct Header {
    std::uint8_t kind = kKindQem;
    std::uint8_t chunkCodec = kChunkCodecZlib;
    std::uint32_t blockRounds = 1;
    std::uint32_t rounds = 0;
    std::uint32_t numChunks = 0;
    std::uint32_t baseLen = 0;
    // QEM only:
    std::uint16_t idWidth = 2;
    std::uint16_t geomBytes = 4;
    std::uint32_t originalVertices = 0;
    std::uint32_t originalFaces = 0;
    std::uint32_t baseVertices = 0;
    std::uint32_t baseFaces = 0;
    std::uint64_t totalCollapses = 0;
    std::vector<std::uint32_t> topoLens;  // QEM: per round topology bytes
    std::vector<std::uint8_t> nativeHeader;  // QEM: baseMesh+round bytes? no: kind-specific
    std::vector<std::uint8_t> base;
};

namespace detail {
inline void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}
inline void putU64(std::vector<std::uint8_t>& out, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
}
inline bool getU32(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint32_t& v) {
    if (off + 4 > in.size()) return false;
    v = 0;
    for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(in[off + static_cast<std::size_t>(i)]) << (8 * i);
    off += 4;
    return true;
}
inline bool getU64(const std::vector<std::uint8_t>& in, std::size_t& off, std::uint64_t& v) {
    if (off + 8 > in.size()) return false;
    v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(in[off + static_cast<std::size_t>(i)]) << (8 * i);
    off += 8;
    return true;
}
inline std::vector<std::uint8_t> zlibCompress(const std::vector<std::uint8_t>& raw) {
    uLongf cap = compressBound(static_cast<uLong>(raw.size()));
    std::vector<std::uint8_t> out(4 + cap);
    uLongf len = cap;
    compress2(out.data() + 4, &len, raw.data(), static_cast<uLong>(raw.size()), Z_BEST_COMPRESSION);
    std::memcpy(out.data(), &len, 4);  // written below
    std::vector<std::uint8_t> result;
    result.reserve(4 + len);
    // [u32 rawLen][u32 compLen][zlib] kept self-describing
    putU32(result, static_cast<std::uint32_t>(raw.size()));
    putU32(result, static_cast<std::uint32_t>(len));
    result.insert(result.end(), out.begin() + 4, out.begin() + 4 + static_cast<std::ptrdiff_t>(len));
    return result;
}
inline bool zlibDecompress(const std::vector<std::uint8_t>& enc,
                           std::vector<std::uint8_t>& raw, std::string* error) {
    if (enc.size() < 8) { if (error) *error = "PRA1: chunk too small"; return false; }
    std::size_t off = 0; std::uint32_t rawLen = 0, compLen = 0;
    if (!getU32(enc, off, rawLen) || !getU32(enc, off, compLen) ||
        off + compLen != enc.size()) {
        if (error) *error = "PRA1: chunk length mismatch";
        return false;
    }
    raw.resize(rawLen);
    uLongf outLen = rawLen;
    const int zret = uncompress(raw.data(), &outLen, enc.data() + off, compLen);
    if (zret != Z_OK || outLen != rawLen) {
        if (error) *error = "PRA1: chunk inflate failed";
        raw.clear();
        return false;
    }
    return true;
}
}  // namespace detail

inline std::vector<std::uint8_t> serializeHeaderRow(const Header& h) {
    std::vector<std::uint8_t> out;
    out.insert(out.end(), kRaMagic, kRaMagic + 4);
    out.push_back(h.kind);
    out.push_back(h.chunkCodec);
    out.push_back(static_cast<std::uint8_t>(h.blockRounds & 0xffu));
    out.push_back(static_cast<std::uint8_t>((h.blockRounds >> 8) & 0xffu));
    detail::putU32(out, h.rounds);
    detail::putU32(out, h.numChunks);
    detail::putU32(out, static_cast<std::uint32_t>(h.nativeHeader.size()));
    detail::putU32(out, static_cast<std::uint32_t>(h.base.size()));
    detail::putU32(out, static_cast<std::uint32_t>(h.topoLens.size()));
    for (std::uint32_t v : h.topoLens) detail::putU32(out, v);
    if (h.kind == kKindQem) {
        out.push_back(static_cast<std::uint8_t>(h.idWidth & 0xffu));
        out.push_back(static_cast<std::uint8_t>((h.idWidth >> 8) & 0xffu));
        out.push_back(static_cast<std::uint8_t>(h.geomBytes & 0xffu));
        out.push_back(static_cast<std::uint8_t>((h.geomBytes >> 8) & 0xffu));
        detail::putU32(out, h.originalVertices);
        detail::putU32(out, h.originalFaces);
        detail::putU32(out, h.baseVertices);
        detail::putU32(out, h.baseFaces);
        detail::putU64(out, h.totalCollapses);
    }
    out.insert(out.end(), h.nativeHeader.begin(), h.nativeHeader.end());
    out.insert(out.end(), h.base.begin(), h.base.end());
    return out;
}

inline bool parseHeaderRow(const std::vector<std::uint8_t>& in, Header* h,
                           std::string* error) {
    auto fail = [&](const char* m) { if (error) *error = m; return false; };
    if (in.size() < 20 || std::memcmp(in.data(), kRaMagic, 4) != 0) return fail("PRA1: bad header magic");
    std::size_t off = 4;
    h->kind = in[off++];
    h->chunkCodec = in[off++];
    h->blockRounds = static_cast<std::uint32_t>(in[off]) |
                     (static_cast<std::uint32_t>(in[off + 1]) << 8);
    off += 2;
    std::uint32_t headerLen = 0, topoCount = 0;
    if (!detail::getU32(in, off, h->rounds) || !detail::getU32(in, off, h->numChunks) ||
        !detail::getU32(in, off, headerLen) || !detail::getU32(in, off, h->baseLen) ||
        !detail::getU32(in, off, topoCount)) {
        return fail("PRA1: truncated header");
    }
    if (h->kind != kKindQem && h->kind != kKindConn) return fail("PRA1: bad kind");
    if (h->chunkCodec != kChunkCodecZlib) return fail("PRA1: bad chunk codec");
    if (h->blockRounds == 0 || h->numChunks == 0) return fail("PRA1: bad block/chunk count");
    h->topoLens.resize(topoCount);
    for (auto& v : h->topoLens) {
        if (!detail::getU32(in, off, v)) return fail("PRA1: truncated topo lens");
    }
    if (h->kind == kKindQem) {
        if (off + 4 > in.size()) return fail("PRA1: truncated qem meta");
        h->idWidth = static_cast<std::uint16_t>(in[off] | (in[off + 1] << 8)); off += 2;
        h->geomBytes = static_cast<std::uint16_t>(in[off] | (in[off + 1] << 8)); off += 2;
        if (!detail::getU32(in, off, h->originalVertices) ||
            !detail::getU32(in, off, h->originalFaces) ||
            !detail::getU32(in, off, h->baseVertices) ||
            !detail::getU32(in, off, h->baseFaces) ||
            !detail::getU64(in, off, h->totalCollapses)) {
            return fail("PRA1: truncated qem meta");
        }
    }
    if (off + headerLen + h->baseLen != in.size()) return fail("PRA1: header lengths mismatch");
    h->nativeHeader.assign(in.begin() + static_cast<std::ptrdiff_t>(off),
                           in.begin() + static_cast<std::ptrdiff_t>(off + headerLen));
    off += headerLen;
    h->base.assign(in.begin() + static_cast<std::ptrdiff_t>(off), in.end());
    return true;
}

struct WriteSpec {
    Header header;
    std::vector<std::vector<std::uint8_t>> chunks;  // raw chunk bytes
};

inline bool writeRaArtifact(const std::string& path, const WriteSpec& spec,
                            std::string* error) {
    auto fail = [&](const std::string& m) { if (error) *error = m; return false; };
    try {
        Header h = spec.header;
        h.numChunks = static_cast<std::uint32_t>(spec.chunks.size());
        const std::vector<std::uint8_t> headerRow = serializeHeaderRow(h);
        std::vector<std::shared_ptr<arrow::Array>> arrays;
        arrow::BinaryBuilder builder;
        auto append = [&](const std::vector<std::uint8_t>& bytes) {
            if (!builder.Append(bytes.data(), static_cast<std::int32_t>(bytes.size())).ok()) {
                throw std::runtime_error("PRA1: append failed");
            }
        };
        append(headerRow);
        for (const auto& chunk : spec.chunks) append(detail::zlibCompress(chunk));
        std::shared_ptr<arrow::Array> array;
        if (!builder.Finish(&array).ok()) throw std::runtime_error("PRA1: builder finish failed");
        auto schema = arrow::schema({arrow::field("paper_chunk", arrow::binary())});
        auto table = arrow::Table::Make(schema, {array}, static_cast<std::int64_t>(array->length()));
        auto sinkResult = arrow::io::FileOutputStream::Open(path);
        if (!sinkResult.ok()) throw std::runtime_error("PRA1: cannot open output");
        auto sink = sinkResult.ValueOrDie();
        parquet::WriterProperties::Builder pb;
        pb.compression(parquet::Compression::UNCOMPRESSED);
        pb.disable_statistics();
        pb.disable_dictionary();
        pb.version(parquet::ParquetVersion::PARQUET_2_6);
        pb.data_pagesize(1024);
        pb.created_by("PMSF page-ra v1");
        auto props = pb.build();
        auto arrowProps = parquet::ArrowWriterProperties::Builder().build();
        auto writerResult = parquet::arrow::FileWriter::Open(
            *schema, arrow::default_memory_pool(), sink, props, arrowProps);
        if (!writerResult.ok()) throw std::runtime_error("PRA1: cannot create writer");
        auto writer = std::move(writerResult).ValueOrDie();
        auto st = writer->WriteTable(*table, 1);
        if (!st.ok()) throw std::runtime_error("PRA1: WriteTable failed: " + st.ToString());
        if (!writer->Close().ok()) throw std::runtime_error("PRA1: close failed");
    } catch (const std::exception& e) {
        return fail(e.what());
    }
    if (error) error->clear();
    return true;
}

/// Page-level reader: open metadata, read only the header RG + first
/// `chunkCount` chunk RGs; bytesRead = selected RG compressed sizes + footer.
class RaReader {
  public:
    explicit RaReader(const std::string& path) : path_(path) {
        auto infile = arrow::io::ReadableFile::Open(path);
        if (!infile.ok()) throw std::runtime_error("PRA1: cannot open " + path);
        file_ = *infile;
        auto openResult = parquet::arrow::OpenFile(file_, arrow::default_memory_pool());
        if (!openResult.ok()) throw std::runtime_error("PRA1: not a parquet RA file");
        reader_ = std::move(openResult).ValueOrDie();
        parquet_ = parquet::ParquetFileReader::OpenFile(path, false);
        if (!parquet_) throw std::runtime_error("PRA1: cannot open parquet metadata");
        metadata_ = parquet_->metadata();
        const auto totalRgs = metadata_->num_row_groups();
        if (totalRgs < 1) throw std::runtime_error("PRA1: no row groups");
        std::uint64_t allRgBytes = 0;
        for (int i = 0; i < totalRgs; ++i) {
            allRgBytes += static_cast<std::uint64_t>(
                metadata_->RowGroup(i)->total_compressed_size());
        }
        std::uint64_t fileSize = 0;
        try { fileSize = std::filesystem::file_size(path); } catch (...) { fileSize = 0; }
        footerBytes_ = fileSize > allRgBytes ? fileSize - allRgBytes : 0;
        readRowGroup(0, &headerRow_);
        if (!parseHeaderRow(headerRow_, &header_, &error_)) {
            throw std::runtime_error("PRA1: " + error_);
        }
    }

    const Header& header() const { return header_; }
    std::uint64_t footerBytes() const { return footerBytes_; }

    /// Read prefix chunks [0, chunkCount); chunkCount clamped to numChunks.
    std::uint64_t readPrefix(std::uint32_t chunkCount,
                             std::vector<std::vector<std::uint8_t>>* chunks) const {
        chunks->clear();
        if (chunkCount > header_.numChunks) chunkCount = header_.numChunks;
        std::uint64_t bytesRead = 0;
        if (metadata_->num_row_groups() < 1 + static_cast<int>(chunkCount)) {
            throw std::runtime_error("PRA1: truncated chunk rows");
        }
        bytesRead += static_cast<std::uint64_t>(metadata_->RowGroup(0)->total_compressed_size());
        for (std::uint32_t i = 0; i < chunkCount; ++i) {
            std::vector<std::uint8_t> row;
            readRowGroup(static_cast<int>(i + 1), &row);
            std::vector<std::uint8_t> raw;
            std::string error;
            if (!detail::zlibDecompress(row, raw, &error)) {
                throw std::runtime_error("PRA1: " + error);
            }
            chunks->push_back(std::move(raw));
            bytesRead += static_cast<std::uint64_t>(
                metadata_->RowGroup(static_cast<int>(i + 1))->total_compressed_size());
        }
        bytesRead_ = bytesRead + footerBytes_;
        rowGroupsRead_ = 1 + chunkCount;
        return bytesRead_;
    }

    std::uint64_t bytesRead() const { return bytesRead_; }
    std::uint32_t rowGroupsRead() const { return rowGroupsRead_; }

    /// Concatenate native header + prefix chunks (CONN profile).
    std::vector<std::uint8_t> connPrefixPayload(std::uint32_t chunkCount,
                                                std::uint64_t* bytesRead) const {
        std::vector<std::vector<std::uint8_t>> chunks;
        *bytesRead = readPrefix(chunkCount, &chunks);
        std::vector<std::uint8_t> out = header_.nativeHeader;
        for (const auto& c : chunks) out.insert(out.end(), c.begin(), c.end());
        return out;
    }

  private:
    void readRowGroup(int rg, std::vector<std::uint8_t>* row) const {
        std::shared_ptr<arrow::Table> table;
        auto st = reader_->ReadRowGroup(rg, &table);
        if (!st.ok()) throw std::runtime_error("PRA1: ReadRowGroup failed");
        if (!table || table->num_columns() != 1 || table->num_rows() != 1) {
            throw std::runtime_error("PRA1: malformed RA row");
        }
        auto arr = std::static_pointer_cast<arrow::BinaryArray>(
            table->column(0)->chunk(0));
        const auto view = arr->GetView(0);
        row->assign(reinterpret_cast<const std::uint8_t*>(view.data()),
                    reinterpret_cast<const std::uint8_t*>(view.data()) + view.size());
    }

    std::string path_;
    std::shared_ptr<arrow::io::ReadableFile> file_;
    std::unique_ptr<parquet::arrow::FileReader> reader_;
    std::shared_ptr<parquet::ParquetFileReader> parquet_;
    std::shared_ptr<parquet::FileMetaData> metadata_;
    std::vector<std::uint8_t> headerRow_;
    Header header_;
    std::string error_;
    std::uint64_t footerBytes_ = 0;
    mutable std::uint64_t bytesRead_ = 0;
    mutable std::uint32_t rowGroupsRead_ = 0;
};

}  // namespace page_ra
}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_PAGE_RA_V1_H
