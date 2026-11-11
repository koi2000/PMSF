/*
 * @file ContextLevelTrackingPreprocessor.h
 * @brief Phase 5 重构：把 "每轮主循环开始时 contextPool.setMetadata("level", i)" 抽象为
 *        一等公民的 RoundPreProcessor。PCATM / PCLTTM / RD / APTM / MMSE 都要。
 *
 * 这不是原 PreProcessOperator 体系的 Operator（因为 process 签名是按 mesh 一次性调用），
 * 而是"每轮"粒度的 hook。Phase 5 引入 RoundPreProcessor 是为了让 Builder 可以通过
 * `.withRoundPreprocessor(std::make_shared<ContextLevelTrackingPreprocessor>())` 来注入，
 * 而不再需要 impl/ 的 main 硬编码 cfg.beforeRound lambda。
 */
#pragma once

#include <memory>

namespace MCGAL {
class Mesh;
}

namespace pmsf {

/// @brief "每轮"粒度的 hook 基类。beforeRound(i, mesh) 在第 i 轮主循环开始时被调用。
class RoundPreProcessor {
  public:
    virtual ~RoundPreProcessor() = default;
    virtual void beforeRound(int roundIndex, std::shared_ptr<MCGAL::Mesh> mesh) = 0;
};

/// @brief 每轮把 contextPool 的 "level" meta 设为当前 round index。
///        PCATM/PCLTTM/RD/APTM/MMSE 的 Level2* DegreeN operator 依赖此 meta。
class ContextLevelTrackingPreprocessor : public RoundPreProcessor {
  public:
    ContextLevelTrackingPreprocessor() = default;
    ~ContextLevelTrackingPreprocessor() override = default;

    void beforeRound(int roundIndex, std::shared_ptr<MCGAL::Mesh> mesh) override;
};

}  // namespace pmsf
