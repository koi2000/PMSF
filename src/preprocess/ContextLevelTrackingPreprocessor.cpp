/*
 * @file ContextLevelTrackingPreprocessor.cpp
 */
#include "ContextLevelTrackingPreprocessor.h"

#include "Global.h"        // MCGAL::contextPool（extern ContextPool&）
#include "ContextPool.h"   // ContextValue

namespace pmsf {

void ContextLevelTrackingPreprocessor::beforeRound(int roundIndex,
                                                    std::shared_ptr<MCGAL::Mesh> /*mesh*/) {
    MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(roundIndex));
}

}  // namespace pmsf
