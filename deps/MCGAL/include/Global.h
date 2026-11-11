#ifndef GLOBAL_H
#define GLOBAL_H
#include "ContextPool.h"
#include "ThreadPool.h"
namespace MCGAL {
extern ContextPool& contextPool;
extern ThreadPool& threadPool;
}

#endif