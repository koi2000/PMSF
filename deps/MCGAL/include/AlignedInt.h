#ifndef ALIGNEDINT_H
#define ALIGNEDINT_H
#include "stdint.h"
#include <stdlib.h>

template <typename T, size_t alignment = alignof(T)> struct AlignedInt {
    static_assert(alignment >= sizeof(T), "Alignment must be greater than or equal to size of T.");
    union {
        char padding[alignment];
        T value;
    };
};
using AlignedIntType = AlignedInt<int, 64>;

#endif