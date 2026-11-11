/*
 * @file PPMCFormatConstants.h
 * @brief PPMC 二进制落盘格式的共享常量（编解码两端必须一致）。
 *
 * 这些数字是**格式的一部分**：改了就必须同时改编码端与解码端，且改变所有已产出
 * 的 PPMC 二进制产物的可读性。格式里没有版本号字段，所以只能靠这个头文件把
 * 「唯一定义处」固定下来 —— 过去两端各写字面量，已经出过一次不一致。
 */
#ifndef PMSF_PPMC_FORMAT_CONSTANTS_H
#define PMSF_PPMC_FORMAT_CONSTANTS_H

/// @brief base mesh 的「面度数 - 3」字段位宽。
///
/// 3 位只能表示 degree 3..10。实测 LetterA（Thingi10K，含高次面）的基础网格会
/// 出现 degree > 10 的面，编码端的 `assert(i_code < 1 << 3)` 直接中断 —— 也就是
/// 这个格式**承载不了**该网格，而不是算法算错了。故加宽到 4 位（degree 3..18）。
#define NB_BITS_FACE_DEGREE_BASE_MESH 4

#endif  // PMSF_PPMC_FORMAT_CONSTANTS_H
