/*
 * @Author: koi
 * @Date: 2025-09-04 16:03:58
 * @Description:
 */

#ifndef DEFINE_H
#define DEFINE_H

#define CHECK_TRUE(expr)                                                                                                                             \
    do {                                                                                                                                             \
        if (!(expr)) {                                                                                                                               \
            return false;                                                                                                                            \
        }                                                                                                                                            \
    } while (0)

#endif