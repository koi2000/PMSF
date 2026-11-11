cmake_minimum_required(VERSION 3.10)
project(MCGAL)


# 2026-05-04 性能修复：原先硬编码 `-O0 -g`，覆盖顶层 Release (-O3) 设置，
#   导致 MCGAL_static 走 O0 —— `Halfedge::find_prev` / `Facet::facet_degree` /
#   `Vertex::halfedge_iterator::operator++` 全部以函数调用方式存在，
#   perf 显示这几项合计占 RD/MMSE/APTM 40%+ CPU。顶层 CMake 以变量 `DEBUG` 控制，
#   此处对齐：DEBUG 下保留 O0，其它场景继承顶层 -O3 -g (PROFILE 还带 -fno-omit-frame-pointer)。
if(DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wno-unused-result -lpthread")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wno-unused-result -lpthread")
endif()

# 收集源文件
file(GLOB MCGAL_SRC 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/openmesh/*.cc"

)

# 设置头文件路径
set(MCGAL_INCLUDE_DIR 
    "${CMAKE_CURRENT_SOURCE_DIR}/include",
    "${CMAKE_CURRENT_SOURCE_DIR}/include/openmesh"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/openmesh/System"
    "/usr/local/include/OpenMesh/include/"
)

# find_package(OpenMP REQUIRED)

set(LIBS_DEBUG libOpenMeshCore.a libOpenMeshCore.so libOpenMeshTools.a libOpenMeshTools.so)
set(LIBS_RELEASE libOpenMeshCore.a libOpenMeshCore.so libOpenMeshTools.a libOpenMeshTools.so)

# 静态库
add_library(MCGAL_static STATIC ${MCGAL_SRC})
target_include_directories(MCGAL_static PUBLIC ${MCGAL_INCLUDE_DIR})
target_link_libraries(MCGAL_static ${LIBS_DEBUG})


# 动态库
# 注：本仓库所有可执行目标均链接 MCGAL_static，MCGAL_shared 当前没有任何下游使用者。
# 由于系统安装的 libOpenMeshCore.a 未使用 -fPIC 编译，将其链入 SHARED 目标会触发
#   "relocation R_X86_64_32 against `.rodata.str1.1' can not be used when making a shared object"
# 链接错误，导致 cmake --build 默认构建直接失败。
# 这里把 MCGAL_shared 标记为 EXCLUDE_FROM_ALL：仍然保留目标定义（如需可显式 build），
# 但 `make all` / `cmake --build .` 不再尝试构建它，避免阻断整个工程的编译。
add_library(MCGAL_shared SHARED EXCLUDE_FROM_ALL ${MCGAL_SRC})
target_include_directories(MCGAL_shared PUBLIC ${MCGAL_INCLUDE_DIR})
target_link_libraries(MCGAL_shared ${LIBS_DEBUG})


# 源码方式就是直接用上面 static/shared 的target