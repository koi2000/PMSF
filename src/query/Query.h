/*
 * @file Query.h
 * @brief 统一随机访问 API：`Query{roi, max_lod}`（设计文档 §2.5 / 论文 §3.5.3）。
 *
 * 目标：ROI × LOD 二维随机访问是**所有算法的一等能力**，而不是某个算法的专有
 * 产物。查询编译成两个谓词：
 *
 *   roi     → partition / 组 谓词（论文式 (1) 的 bbox 谓词，当前以 group id 表达）
 *   max_lod → lod_index 的 `lod_id < k` 谓词（论文式 (2)(3) 的前缀截断）
 *
 * 两个谓词都通过 `DeserializeOperator::setQueryRegion` / `setQueryMaxLod` 下达，
 * 因此对"支持随机访问的后端"（列式）真正生效，对连续字节流后端是 no-op
 * （`supportsRandomAccess()` 返回 false，调用方可据此降级为全量解码）。
 *
 * 契约：**截断即前缀**。只解码前 k 个 LOD 所读取的符号量，恰为 lod_index 表前 k
 * 行的 offsets 之和（`tests/test_truncation_prefix.cpp` 固化此性质）。
 */
#ifndef PMSF_QUERY_QUERY_H
#define PMSF_QUERY_QUERY_H

#include "core/Backend.h"


#include <memory>
#include <string>
#include <vector>

namespace MCGAL {
class Mesh;
}  // namespace MCGAL

namespace pmsf {

/// @brief 空间区域选择。空 = 全部区域。
struct Roi {
    /// 分区 / 空间簇 id 列表。空表示不限制。
    std::vector<int> groupIds;
    /// 包围盒形式（论文式 (1) 的 bbox 谓词）。hasBbox=false 时忽略。
    bool   hasBbox = false;
    double bboxMin[3] = {0.0, 0.0, 0.0};
    double bboxMax[3] = {0.0, 0.0, 0.0};

    bool isEmpty() const { return groupIds.empty() && !hasBbox; }
};

/// @brief 一次随机访问查询。
struct Query {
    Roi roi;
    /// 最大 LOD（不含）。<0 表示全部 LOD —— 即完整渐进解码。
    int maxLod = -1;

    static Query full() { return Query{}; }
    static Query byLod(int k) { Query q; q.maxLod = k; return q; }
    static Query byRegion(int groupId, int k = -1) {
        Query q;
        q.roi.groupIds.push_back(groupId);
        q.maxLod = k;
        return q;
    }
};

/// @brief 查询结果。
struct QueryResult {
    std::shared_ptr<MCGAL::Mesh> mesh;
    /// 后端是否真正执行了谓词下推。false 表示退化为全量解码后再切片。
    bool predicatePushedDown = false;
    /// 实际参与解码的区域 id（predicatePushedDown=false 时为空）。
    std::vector<int> decodedRegions;
};

/// @brief 执行一次 `Query{roi, max_lod}`。
///
/// 读 metadata 里的算法名 → 从注册表再实例化解压组件 → 把查询编译成谓词下达 →
/// 跑统一解码循环。**框架层没有一行算法代码**：算法名只用于查表。
///
/// @param artifactPath 压缩产物基础名（与 `PMSFCompress` 的 output 一致）
/// @param q            查询
/// @param algorithmName 产物的算法名（= 压缩时的 info().name）。Phase 3 会把
///        它写进 metadata 表，届时本参数可从产物自描述获得；当前由调用方给出。
/// @param backend      产物后端（columnar / unified / binary）
/// @param outputPath   非空时把结果写成 .off
QueryResult runQuery(const std::string& artifactPath,
                     const Query& q,
                     const std::string& algorithmName,
                     Backend backend = Backend::Columnar,
                     const std::string& outputPath = {});

}  // namespace pmsf

#endif  // PMSF_QUERY_QUERY_H
