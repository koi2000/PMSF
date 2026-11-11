// CGAL 5.2 → 5.5/5.6 兼容 shim（**外部注入，不改参考实现源码**）
//
// 背景：MEPP2 的 Compression_Valence 是为 CGAL 5.2 写的，其中
//   Compression_Valence_Component.inl:6531 用三参位置形式调用
//     CGAL::copy_face_graph(_pMesh, mesh_copy, std::inserter(v2v, v2v.end()));
// 把 v2v 输出迭代器直接当第三个实参。CGAL 从 5.3 起把 copy_face_graph 改成
// 命名参数接口（internal_np::get_parameter(np, tag)），而 parameters::get_parameter
// 只接受 Named_function_parameters，于是原始迭代器没有可匹配的重载 ⇒ 编译失败。
//
// 本头文件补上缺失的那两个重载，语义与 5.2 的位置参数形式**等价**：
//   · 输出迭代器 tag  → 原样返回该迭代器（CGAL 自己的实现会用它做 v2v 记录）
//   · 其余 tag        → 返回 Param_not_found，交给 choose_parameter 取默认值
//
// 这样 CGAL 自带的 copy_face_graph 实现照常工作，我们**没有**重写它的复制逻辑
// ——重写会引入"仪表本身可能是错的"风险（round-trip 结论将不可信）。
//
// 用法：-include /tmp/p11/cgal52_compat.h（必须早于 copy_face_graph.h 被解析；
// 由于该头在函数体内用 `using parameters::get_parameter;` 做限定查找，
// 重载只要在实例化点之前可见即可，-include 满足）。

#pragma once

#include <iterator>

#include <CGAL/Named_function_parameters.h>

namespace CGAL {
namespace parameters {

// 兜底：任何未识别的 tag 都回 Param_not_found（等价于"该命名参数未提供"）。
// 只对 std::insert_iterator 生效，不会干扰 Named_function_parameters 的正常解析。
template <typename Map, typename Query_tag>
inline internal_np::Param_not_found
get_parameter(const std::insert_iterator<Map> &, Query_tag) {
  return internal_np::Param_not_found();
}

// 输出迭代器 tag：原样返回，等价于 CGAL 5.2 把第三个位置参数当作
// vertex_to_vertex_output_iterator 的行为。
template <typename Map>
inline std::insert_iterator<Map>
get_parameter(const std::insert_iterator<Map> &it,
              internal_np::vertex_to_vertex_output_iterator_t) {
  return it;
}

} // namespace parameters
} // namespace CGAL
