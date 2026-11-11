// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "Global.h"
#include "Mesh.h"
#include <climits>
#include <memory>
#include <unordered_map>

namespace pmsf {

// A temporary, separately allocated mesh for simulate-before-encoding stages.
// No production pool is reused or freed. IDs are never written to a stream.
class ConquestMeshClone {
 public:
    ConquestMeshClone(MCGAL::Mesh& source,std::size_t finalVertices):mesh_(std::make_unique<MCGAL::Mesh>()) {
        if(finalVertices>static_cast<std::size_t>((INT_MAX-4096)/24))throw std::runtime_error("conquest simulation too large");
        auto& pool=MCGAL::contextPool;
        pool.initPoolSize(static_cast<int>(pool.vpools.size()+1));
        id_=pool.registerPool(static_cast<int>(finalVertices+64),static_cast<int>(24*finalVertices+4096),static_cast<int>(8*finalVertices+1024));
        try {
        mesh_->setMeshId(id_);mesh_->enableQuantization=source.enableQuantization;
        mesh_->i_nbQuantBits=source.i_nbQuantBits;mesh_->f_quantStep=source.f_quantStep;
        mesh_->bboxMin=source.bboxMin;mesh_->bboxMax=source.bboxMax;
        std::unordered_map<MCGAL::Vertex*,MCGAL::Vertex*> map;
        for(auto* v:source.vertices())if(v&&!v->isRemoved()) {
            auto* copy=pool.allocateVertexFromPool(id_,v->x(),v->y(),v->z());mesh_->add_vertex(copy);map.emplace(v,copy);
        }
        for(auto* f:source.faces())if(f&&!f->isRemoved()) {
            std::vector<MCGAL::Vertex*> ring;auto* first=f->proxyHalfedge();auto* e=first;
            if(!first)throw std::runtime_error("conquest clone: missing face");
            do{if(!e||ring.size()>6)throw std::runtime_error("conquest clone: broken face");ring.push_back(map.at(e->vertex()));e=e->next();}while(e!=first);
            mesh_->add_face(pool.allocateFaceFromPool(id_,ring));
        }
        } catch (...) {release();throw;}
    }
    ~ConquestMeshClone() {release();}
    ConquestMeshClone(const ConquestMeshClone&)=delete;
    ConquestMeshClone& operator=(const ConquestMeshClone&)=delete;
    MCGAL::Mesh* get() const{return mesh_.get();}
 private:
    void release() noexcept {
        mesh_.reset();if(id_<0)return;
        auto& p=MCGAL::contextPool;
        delete[] p.vpools[id_];p.vpools[id_]=nullptr;
        if(p.hraw_[id_]){for(int j=0;j<p.hindexs[id_];++j)std::destroy_at(&p.hpools[id_][j]);::operator delete[](p.hpools[id_]);}
        else delete[] p.hpools[id_];p.hpools[id_]=nullptr;
        if(p.fraw_[id_]){for(int j=0;j<p.findexs[id_];++j)std::destroy_at(&p.fpools[id_][j]);::operator delete[](p.fpools[id_]);}
        else delete[] p.fpools[id_];p.fpools[id_]=nullptr;
    }
    std::unique_ptr<MCGAL::Mesh> mesh_;
    int id_=-1;
};
} // namespace pmsf
