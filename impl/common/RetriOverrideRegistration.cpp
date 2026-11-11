/*
 * @file RetriOverrideRegistration.cpp
 * @brief 把「协议覆盖」的实现注册进 src/ 的接缝 —— **显式的一个 TU**。
 *
 * ## 为什么需要这个文件
 *
 * `RetriOverride.h` 里有一个头文件级的静态初始化块
 * （`g_retriOverridesRegistered`），它调用
 * `pmsf::registerRetriCompressOverride` / `registerRetriDecompressOverride` /
 * `registerRetriDeserializerFactory` 把协议覆盖挂到 `src/` 的接缝上。
 *
 * 头文件里的静态初始化**只在该头被某个 TU 包含时才存在**。过去这件事靠"碰巧"：
 * 一批 legacy 入口文件（`impl/<ALGO>/PMSFEntry.cpp` 等）都 include 了它，
 * 所以注册一直在。2026-09-16 把这些已死的 legacy 入口从构建里摘掉后，
 * 最后一个包含者消失 —— 注册随之消失，且**没有任何报错**：
 * 协议覆盖静默失效，所有走协议的算法回落到标准管线，
  产物体积暴涨（实测 qem/bunny 285815 → 338430，+18%；pcatm/bunny +168%）。
 *
 * 所以这里放一个**没有任何逻辑、只为副作用存在**的 TU：注册不再依赖
 * "谁碰巧包含了这个头"，而是依赖"这个文件在构建里"。摘掉它就会立刻在
 * 回归门禁上炸出来，而不是悄悄改变产物。
 *
 * 这条约束值得记住：**靠头文件静态初始化传递的注册是隐式依赖**，
 * 换 TU 集合就消失。凡是被别处依赖的注册，都应当有明确的落点 TU。
 */
#include "common/RetriOverride.h"
