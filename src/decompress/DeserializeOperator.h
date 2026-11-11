/*
 * @Author: koi
 * @Date: 2025-08-29 22:44:22
 * @Description:
 */
#ifndef DeserializeOperator_H
#define DeserializeOperator_H

#include "../header/PMSFHeader.h"
#include "core/Symbol.h"
#include "Mesh.h"
#include "decompress/SymbolReadOperator.h"
#include "reconstruct/ReconstructOpertator.h"
#include "storage/PMSFPayloadDescriptorRegistry.h"
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief 压缩操作符
 * @details 压缩操作符是用于压缩网格的抽象类，有一个困难点在于很多东西不知道怎么去存，不知道哪些要存，不知道怎么去存
 * 比如在现有的统一化设计方案里，需要先去三角化然后插入中心点
 * 是否可以尝试将信息全部存储在halfedge上，第一个是isAdded，来标识是否为新增，第二个为isSplittable，同时记录点的位置信息，但这样存在一个缺点，
 * 半边的数量是远大于面片的数量的，这样会导致文件大小增加，最好的方式是：
 * 将isAdded信息attach到边而不是半边上，这样会让bit数量减少一半
 */
class DeserializeOperator {
  public:
    DeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator, std::shared_ptr<ReconstructOperator> reconstructOperator)
        : symbolReadOperator_(symbolReadOperator), reconstructOperator_(reconstructOperator) {}

    virtual void init(std::string path);

    void init(char* buffer);

    virtual void clearPMSFSymbolContext();

    virtual void setPMSFSymbolTable(pmsf::PMSFSymbolTable table);

    virtual void setPMSFSymbolPayloadTable(pmsf::PMSFSymbolPayloadTable table);

    // ---- 随机访问谓词（设计文档 §2.5 / 论文式 (1)(2)(3)）-------------------
    //
    // Query{roi, max_lod} 编译成的两个谓词，落到解压器上。基类默认 no-op：
    // 不支持随机访问的后端（BasicSerializeOperator 的连续字节流）忽略即可；
    // 列式后端覆写为真正的 row-group / 行级过滤。
    //
    // regionId < 0 表示「全部区域」；maxLod < 0 表示「全部 LOD」。两者语义都是
    // 「截断即前缀」：只解码前 maxLod 轮，其读取量恰为前 maxLod 行的 offsets 之和。
    /// @brief 取本解压器持有的符号读取算子。
    ///        供解压侧的 afterBaseMesh 回调把 header 里的算法级开关下传
    ///        （例如把预测开关交给算法自己的 SymbolReadOperator）——
    ///        这样算法不必另外持有 reader 引用，也就能做到"解压侧零算法代码"。
    std::shared_ptr<SymbolReadOperator> symbolReadOperator() const {
        return symbolReadOperator_;
    }

    virtual void setQueryRegion(int /*regionId*/) {}
    virtual void setQueryMaxLod(int /*maxLod*/) {}

    /// @brief 本次解码的轮数上限（`max_lod` 谓词），< 0 表示不限。
    ///
    /// 这是「截断即前缀」的落点：解码循环跑前 k 轮，等价于全量解码在第 k 轮
    /// 结束时的中间状态；且各轮在产物内首尾相接，所以「跑前 k 轮」读取的字节
    /// 恰是前 k 轮的 offsets 之和 —— 读取量本身也是前缀。
    ///
    /// 放在基类上是为了让**任意算法、任意后端**都能享受同一语义：解压主循环
    /// (runDecompress) 只问这一个问题，不关心谓词是怎么被后端落实的。
    /// @brief 产物是否为 boundary-stage storage V3 (mode71) payload.
    ///        Self-describing decode: once true the decoder enables the stable-ID
    ///        registry without requiring the PMSF_BND_V3 environment variable.
    virtual bool isBoundaryV3Payload() const { return false; }

    /// @brief Decoupled boundary schedule: apply all per-round boundary inverses AFTER
    ///        the whole interior round loop (boundary-first decode). Default no-op.
    virtual void finalizeBoundaryInverses(int /*rounds*/) {}

    virtual int queryMaxLod() const { return -1; }

    /// @brief 本后端是否支持 (roi, lod) 二维随机访问。
    virtual bool supportsRandomAccess() const { return false; }

    bool hasPMSFSymbolTable() const {
        return hasPMSFSymbolTable_;
    }

    bool hasPMSFSymbolPayloadTable() const {
        return hasPMSFSymbolPayloadTable_;
    }

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const {
        return hasPMSFSymbolTable_ ? &pmsfSymbolTable_ : nullptr;
    }

    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const {
        return hasPMSFSymbolPayloadTable_ ? &paperSymbolPayloadTable_ : nullptr;
    }

    const pmsf::PMSFSymbolPayloadRecord* findPMSFSymbolPayload(int objectId,
                                                                 int partitionId,
                                                                 int lodId,
                                                                 const std::string& payloadName) const;

    const std::vector<std::uint8_t>* findPMSFSymbolPayloadBytes(
        int objectId,
        int partitionId,
        int lodId,
        const std::string& payloadName) const;

    const pmsf::PMSFSymbolPayloadRecord* findPMSFSymbolPayloadByRole(
        pmsf::PMSFPayloadRole role,
        int objectId = 0,
        int partitionId = 0,
        int lodId = 0) const;

    const std::vector<std::uint8_t>* findPMSFSymbolPayloadBytesByRole(
        pmsf::PMSFPayloadRole role,
        int objectId = 0,
        int partitionId = 0,
        int lodId = 0) const;

    const pmsf::PMSFSymbolPayloadRecord* findFirstPMSFSymbolPayloadByCodecSuffix(
        const std::string& codecSuffix) const;

    const pmsf::PMSFSymbolPayloadRecord*
    findFirstPMSFSymbolPayloadByRoleAndCodecSuffix(
        pmsf::PMSFPayloadRole role,
        const std::string& codecSuffix) const;

    bool restoreBufferFromPMSFPayload(const std::string& path,
                                       const std::string& payloadName);

    bool restoreLegacySingleFileBufferFromPMSFPayload(const std::string& path);

    void propagatePMSFSymbolContextTo(
        const std::shared_ptr<DeserializeOperator>& inner) const;

    const std::string& paperSymbolSidecarPath() const {
        return paperSymbolSidecarPath_;
    }

    const std::string& paperSymbolPayloadBlobPath() const {
        return paperSymbolPayloadBlobPath_;
    }

    virtual void setSeeds(std::vector<MCGAL::Halfedge*> seeds) {
        seeds_ = seeds;
    };

    void setMesh(std::shared_ptr<MCGAL::Mesh> mesh) {
        mesh_ = mesh;
        reconstructOperator_->init(mesh);
    }

    void deserializeInt(int& val);

    virtual void deserializeHeader(PMSFHeader& header) {
        header.deserialize(buffer_, dataOffset_);
        header_ = header;
    };

    virtual std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() = 0;

    virtual void deserializeOneRound() = 0;

    virtual std::shared_ptr<MCGAL::Mesh> exportMesh() {
        return mesh_;
    };

    virtual PMSFHeader header() {
        return header_;
    }

    /// @brief 原始网格计数（在 init() 之后、deserializeBaseMesh() 之前调用），
    ///        供解码侧按需注册池容量；未覆写的后端返回 (0,0) ⇒ 走内置默认。
    ///        乘数集中在 runDecompress 一处（标定见 MeshRuntime.h 说明）。
    virtual void originalMeshCounts(int& v, int& f) const {
        v = 0;
        f = 0;
    }

    // 注意：不是每条路径都会给 buffer_ 赋值（例如 PomarDeserializeOperator 的
    // init() 只填自有成员 p_data/p_dataOffset）。留未初始化指针会让
    // 基类 deserializeHeader 从野指针读 header —— 实测 pomar 二进制解码的
    // header.round 因此变成布局相关随机值，轮循环空转挂起。初始化为 nullptr
    // 把这类误用从「静默读垃圾」变成「确定的空指针」。
    char* buffer_ = nullptr;
    size_t dataOffset_ = 0;
    size_t pointOffset_ = 0;
    size_t totalSize = 0;
    std::shared_ptr<SymbolReadOperator> symbolReadOperator_;
    std::shared_ptr<ReconstructOperator> reconstructOperator_;
    std::shared_ptr<MCGAL::Mesh> mesh_;
    PMSFHeader header_;
    std::vector<MCGAL::Halfedge*> seeds_;
    pmsf::PMSFSymbolTable pmsfSymbolTable_;
    pmsf::PMSFSymbolPayloadTable paperSymbolPayloadTable_;
    std::string paperSymbolSidecarPath_;
    std::string paperSymbolPayloadBlobPath_;
    bool hasPMSFSymbolTable_ = false;
    bool hasPMSFSymbolPayloadTable_ = false;

  protected:
    void loadPMSFSymbolSidecarIfPresent(const std::string& path);

  private:
    bool readBuffer(std::string path);
};
#endif
