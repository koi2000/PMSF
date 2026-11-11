/*
 * @Author: koi
 * @Date: 2025-08-27 14:51:39
 * @Description: 使用哪种作为error来源，半边还是顶点，也可以是无
 */

#ifndef ERROR_SOURCE_H
#define ERROR_SOURCE_H

enum class ErrorSource {
    None,
    Vertex,
    Halfedge,
};

#endif