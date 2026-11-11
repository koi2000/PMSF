// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "QEMProgressiveStreamsSeam.h"
#include "storage/PaperStreamReaderV2.h"
#include "storage/PaperStreamWriterV2.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace pmsf::qem_paper1997 {

// Mapping only: the physical writer/reader and schema are shared with every
// algorithm. Exact f64 reconstruction positions remain a named geometry stream
// because the existing Cf/Cv physical LIST columns are float32.
inline void writePaperRelations(const std::string& path,
                                const qem_prog::ArtifactStreams& streams,
                                const qem_prog::EncodeStats& stats,
                                bool f64) {
    qem_prog::EncodeOptions encodeOptions; encodeOptions.geomF64=f64;
    qem_prog::EncodeStats framed=stats;
    std::vector<std::uint8_t> own;
    std::string error;
    if (!qem_prog::serializeOwnFormat(streams,encodeOptions,&own,&framed,&error)) {
        throw std::runtime_error("QEM mapping: "+error);
    }
    PaperStreamTable table;
    auto& object=table.object;
    object.algorithm="qem";object.formatVersion=kPaperStreamFormatVersionV2;
    object.schemaMode=PaperSchemaMode::Typed;object.payloadMode=PaperPayloadMode::Typed;
    object.numRounds=stats.rounds;object.numPartitions=1;
    object.totalVertices=stats.originalVertices;object.totalFaces=stats.originalFaces;
    object.codecVersion=1;object.blobLayout=kPaperBlobLayoutQEMProgressive;
    object.schemeVersion=1997;object.baseMeshCodec="qem-progressive/base-header-v1";
    object.boundaryCodec.clear();object.raPrefixUnit="lod_block";
    for (const char* role:{"F","Et","Ef","V"}) {
        object.streams.push_back({role,PaperCellType::Bytes,PaperStreamEncoding::Plain,
            "qem_split_topology",0,true,PaperCellShape::Scalar,
            PaperStreamRelation::Connectivity,PaperStreamSource::Payload});
    }
    for (const char* role:{"Cf","Cv","CfQ","CvQ"}) {
        object.streams.push_back({role,PaperCellType::Bytes,PaperStreamEncoding::Plain,
            f64?"qem_split_geometry_f64":"qem_split_geometry_f32",0,true,PaperCellShape::Vec3,
            PaperStreamRelation::Geometry,PaperStreamSource::Payload});
    }
    for (const auto& spec:std::vector<std::pair<std::string,PaperStreamRelation>>{
        {"qem_round",PaperStreamRelation::Connectivity},
        {"qem_split_topology",PaperStreamRelation::Connectivity},
        {f64?"qem_split_geometry_f64":"qem_split_geometry_f32",PaperStreamRelation::Geometry}}) {
        object.streams.push_back({spec.first,PaperCellType::Bytes,PaperStreamEncoding::Plain,
            "qem-progressive/v1",0,true,PaperCellShape::Scalar,spec.second,PaperStreamSource::Payload});
    }
    PaperPartitionRow partition;
    partition.numVertices=stats.baseVertices;partition.numFaces=stats.baseFaces;
    partition.numLods=stats.rounds;partition.baseMeshCodec=object.baseMeshCodec;
    partition.baseMesh.assign(own.begin(),own.begin()+qem_prog::kHeaderBytes);
    partition.baseMesh.insert(partition.baseMesh.end(),streams.baseMeshStream.begin(),streams.baseMeshStream.end());
    table.partitions.push_back(std::move(partition));
    std::size_t topologyOffset=0,geometryOffset=0;
    std::uint64_t offset=0;
    for (int lod=0;lod<stats.rounds;++lod) {
        PaperLodRow row;row.lodId=lod;
        const auto& descriptor=streams.roundDescriptors.at(lod);
        row.extension["qem_round"]={streams.roundStream.begin()+lod*4,streams.roundStream.begin()+(lod+1)*4};
        row.extension["qem_split_topology"]={streams.topologyStream.begin()+topologyOffset,
            streams.topologyStream.begin()+topologyOffset+descriptor.topologyBytes};
        row.extension[f64?"qem_split_geometry_f64":"qem_split_geometry_f32"]={
            streams.geometryStream.begin()+geometryOffset,streams.geometryStream.begin()+geometryOffset+descriptor.geometryBytes};
        row.payloadByteOffset=offset;
        row.payloadByteLength=4+descriptor.topologyBytes+descriptor.geometryBytes;
        offset+=row.payloadByteLength;
        topologyOffset+=descriptor.topologyBytes;geometryOffset+=descriptor.geometryBytes;
        table.lods.push_back(std::move(row));
    }
    if (topologyOffset!=streams.topologyStream.size()||geometryOffset!=streams.geometryStream.size()) {
        throw std::runtime_error("QEM mapping: per-LOD directory does not span logical streams");
    }
    PaperWriterV2Options options;options.lodsPerRowGroup=8;
    PaperStreamWriterV2().write(table,path,options);
}

inline bool readPaperRelations(const std::string& path,int maxLod,
                               qem_prog::SeamStreams* streams,std::string* error,
                               std::uint64_t* bytesRead=nullptr,
                               std::uint64_t* rowGroupsRead=nullptr,
                               std::vector<std::uint8_t>* declaredHeader=nullptr) {
    try {
        PaperStreamReaderV2 reader(path);
        const auto table=maxLod<0?reader.readAll():reader.readPrefix(0,maxLod);
        if (reader.usedV1Fallback()||reader.isBlobContainer()||table.object.algorithm!="qem"||
            table.object.blobLayout!=kPaperBlobLayoutQEMProgressive||table.object.schemeVersion!=1997||
            table.partitions.size()!=1) throw std::runtime_error("unexpected QEM paper object");
        const auto& base=table.partitions.front().baseMesh;
        qem_prog::Reader headerReader(base.data(),base.size());
        qem_prog::detail_own_format::Header header;
        if (!qem_prog::detail_own_format::parseOwnFormatHeader(&headerReader,&header,error)) return false;
        if (header.originalVertices!=static_cast<std::uint32_t>(table.object.totalVertices)||
            header.originalFaces!=static_cast<std::uint32_t>(table.object.totalFaces)||
            header.rounds!=static_cast<std::uint32_t>(table.object.numRounds)) throw std::runtime_error("QEM object/base counter mismatch");
        const std::size_t expectedRows = maxLod < 0 ? header.rounds :
            std::min<std::uint32_t>(static_cast<std::uint32_t>(maxLod), header.rounds);
        if (table.lods.size() != expectedRows) throw std::runtime_error("QEM requested LOD rows are missing");
        qem_prog::SeamStreams out;
        out.meta.qemVersion=1;out.meta.idWidth=header.idWidth;out.meta.geomBytes=header.geomBytes;
        out.meta.geomF64=(header.flags&qem_prog::kFlagGeomF64)!=0;
        out.meta.originalVertices=header.originalVertices;out.meta.originalFaces=header.originalFaces;
        out.meta.baseVertices=header.baseVertices;out.meta.baseFaces=header.baseFaces;
        out.meta.roundCount=static_cast<int>(table.lods.size());out.meta.totalCollapses=0;
        out.baseMesh={base.begin()+header.headerBytes,base.end()};
        std::uint64_t offset=0;
        for (std::size_t lod=0;lod<table.lods.size();++lod) {
            const auto& row=table.lods[lod];const auto& ext=row.extension;
            const auto& round=ext.at("qem_round");
            const auto& topology=ext.at("qem_split_topology");
            const auto& geometry=ext.at(out.meta.geomF64?"qem_split_geometry_f64":"qem_split_geometry_f32");
            if (row.lodId!=static_cast<int>(lod)||round.size()!=4||row.payloadByteOffset!=offset||
                row.payloadByteLength!=round.size()+topology.size()+geometry.size()) throw std::runtime_error("QEM per-LOD directory mismatch");
            const auto count=static_cast<std::uint32_t>(round[0])|(static_cast<std::uint32_t>(round[1])<<8)|
                (static_cast<std::uint32_t>(round[2])<<16)|(static_cast<std::uint32_t>(round[3])<<24);
            out.meta.totalCollapses+=count;
            out.round.insert(out.round.end(),round.begin(),round.end());
            out.topology.insert(out.topology.end(),topology.begin(),topology.end());
            out.geometry.insert(out.geometry.end(),geometry.begin(),geometry.end());
            offset+=row.payloadByteLength;
        }
        if (maxLod<0 && out.meta.totalCollapses!=static_cast<long long>(header.collapses)) throw std::runtime_error("QEM collapse total mismatch");
        std::vector<std::uint8_t> validated;
        if (!qem_prog::buildOwnFormatFromStreams(out,&validated,error)) return false;
        *streams=std::move(out);
        if (declaredHeader) *declaredHeader={base.begin(),base.begin()+header.headerBytes};
        if(bytesRead)*bytesRead=reader.bytesRead();
        if(rowGroupsRead)*rowGroupsRead=reader.rowGroupsRead();
        return true;
    } catch(const std::exception& exception) {
        if(error)*error=std::string("QEM relational mapping: ")+exception.what();
        return false;
    }
}
}  // namespace pmsf::qem_paper1997
