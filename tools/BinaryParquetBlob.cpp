#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/reader.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("failed to open input: " + path);
    }
    in.seekg(0, std::ios::end);
    const auto size = in.tellg();
    if (size < 0) {
        throw std::runtime_error("failed to stat input: " + path);
    }
    in.seekg(0, std::ios::beg);
    std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
    if (!data.empty()) {
        in.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
        if (!in) {
            throw std::runtime_error("failed to read input: " + path);
        }
    }
    return data;
}

void throwIfNotOk(const arrow::Status& status) {
    if (!status.ok()) {
        throw std::runtime_error(status.ToString());
    }
}

void writeFile(const std::string& path, const std::uint8_t* data, std::size_t size) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("failed to open output: " + path);
    }
    if (size > 0) {
        out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
        if (!out) {
            throw std::runtime_error("failed to write output: " + path);
        }
    }
}

int pack(const std::string& input, const std::string& output, const std::string& format) {
    std::vector<std::uint8_t> payload = readFile(input);

    auto schema = arrow::schema({
        arrow::field("format", arrow::utf8(), false),
        arrow::field("payload_bytes", arrow::int64(), false),
        arrow::field("payload", arrow::binary(), false),
    });

    arrow::StringBuilder formatBuilder;
    arrow::Int64Builder sizeBuilder;
    arrow::BinaryBuilder payloadBuilder;
    throwIfNotOk(formatBuilder.Append(format));
    throwIfNotOk(sizeBuilder.Append(static_cast<std::int64_t>(payload.size())));
    throwIfNotOk(payloadBuilder.Append(payload.data(), static_cast<std::int32_t>(payload.size())));

    std::shared_ptr<arrow::Array> formatArray;
    std::shared_ptr<arrow::Array> sizeArray;
    std::shared_ptr<arrow::Array> payloadArray;
    throwIfNotOk(formatBuilder.Finish(&formatArray));
    throwIfNotOk(sizeBuilder.Finish(&sizeArray));
    throwIfNotOk(payloadBuilder.Finish(&payloadArray));

    auto table = arrow::Table::Make(schema, {formatArray, sizeArray, payloadArray}, 1);
    auto outResult = arrow::io::FileOutputStream::Open(output);
    if (!outResult.ok()) {
        throw std::runtime_error(outResult.status().ToString());
    }
    auto out = outResult.ValueOrDie();

    parquet::WriterProperties::Builder propsBuilder;
    propsBuilder.compression(parquet::Compression::ZSTD);
    propsBuilder.compression_level(3);
    propsBuilder.disable_dictionary();
    auto writerProps = propsBuilder.build();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().store_schema()->build();

    auto writerResult = parquet::arrow::FileWriter::Open(
        *schema, arrow::default_memory_pool(), out, writerProps, arrowProps);
    PARQUET_THROW_NOT_OK(writerResult.status());
    auto writer = std::move(writerResult).ValueOrDie();
    PARQUET_THROW_NOT_OK(writer->WriteTable(*table, 1));
    PARQUET_THROW_NOT_OK(writer->Close());
    throwIfNotOk(out->Close());

    std::cout << "packed_payload_bytes=" << payload.size() << "\n";
    return 0;
}

int unpack(const std::string& input, const std::string& output) {
    auto inResult = arrow::io::ReadableFile::Open(input);
    if (!inResult.ok()) {
        throw std::runtime_error(inResult.status().ToString());
    }
    auto in = inResult.ValueOrDie();
    auto readerResult =
        parquet::arrow::OpenFile(in, arrow::default_memory_pool());
    PARQUET_THROW_NOT_OK(readerResult.status());
    auto reader = std::move(readerResult).ValueOrDie();

    std::shared_ptr<arrow::Table> table;
    PARQUET_THROW_NOT_OK(reader->ReadTable(&table));
    auto payloadColumn = table->GetColumnByName("payload");
    if (!payloadColumn || payloadColumn->num_chunks() == 0) {
        throw std::runtime_error("missing payload column");
    }
    auto payloadArray = std::dynamic_pointer_cast<arrow::BinaryArray>(payloadColumn->chunk(0));
    if (!payloadArray || payloadArray->length() < 1) {
        throw std::runtime_error("empty payload column");
    }

    auto view = payloadArray->GetView(0);
    writeFile(output,
              reinterpret_cast<const std::uint8_t*>(view.data()),
              static_cast<std::size_t>(view.size()));
    std::cout << "unpacked_payload_bytes=" << view.size() << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage:\n"
                  << "  " << argv[0] << " pack <input.bin> <output.parquet> [format]\n"
                  << "  " << argv[0] << " unpack <input.parquet> <output.bin>\n";
        return 1;
    }

    try {
        const std::string mode = argv[1];
        if (mode == "pack") {
            const std::string format = argc >= 5 ? argv[4] : "binary";
            return pack(argv[2], argv[3], format);
        }
        if (mode == "unpack") {
            return unpack(argv[2], argv[3]);
        }
        std::cerr << "unknown mode: " << mode << "\n";
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "BinaryParquetBlob error: " << e.what() << "\n";
        return 2;
    }
}
