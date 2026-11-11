#ifndef PMSF_CORE_PAPER_PROTOCOL_HINT_H
#define PMSF_CORE_PAPER_PROTOCOL_HINT_H

#include "storage/PaperBlobLayoutRegistry.h"

#include <cstdint>

namespace pmsf {

template <typename FacetT>
inline void setPaperProtocolHint(FacetT* facet, std::int32_t layout) {
    if (facet != nullptr) {
        facet->setProtocolHint(layout);
    }
}

template <typename FacetT>
inline void markPaperStrictPatchProtocol(FacetT* facet) {
    setPaperProtocolHint(facet, kPaperBlobLayoutCodec020);
}

template <typename FacetRange>
inline void markPaperStrictPatchProtocolForEach(FacetRange& facets) {
    for (auto* facet : facets) {
        markPaperStrictPatchProtocol(facet);
    }
}

}  // namespace pmsf

#endif  // PMSF_CORE_PAPER_PROTOCOL_HINT_H
