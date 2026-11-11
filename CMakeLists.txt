cmake_minimum_required(VERSION 3.16)

# export CC=/home/koi/install/gcc-12/bin/gcc
# export CXX=/home/koi/install/gcc-12/bin/g++

# ./CudaPartialDecoderTest > output.txt 2>&1
# for local1
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# for local
# set(CMAKE_C_COMPILER "/usr/local/gcc-12/bin/gcc")
# set(CMAKE_CXX_COMPILER "/usr/local/gcc-12/bin/g++")

# set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.3/bin/nvcc")
# for remote
# set(CMAKE_CUDA_COMPILER "/usr/local/cuda-12.2/bin/nvcc")

set(DEBUG OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PROFILE ON CACHE BOOL "Enable profiling mode (O3 + debug info)")

message(STATUS "ENABLE DEBUG: ${DEBUG}")
message(STATUS "ENABLE PROFILE: ${PROFILE}")
message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
project(MeshSimplificationFramework LANGUAGES CXX C)
link_libraries(nghttp2)
# set(CMAKE_CUDA_ARCHITECTURES 89)
set(CMAKE_CXX_STANDARD 20)

if(DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g -Wno-unused-result -lpthread")
    # set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler -rdc=true -arch=compute_89 -code=sm_89 -Xcompiler -std=c++17 -g -G -O0")
elseif(PROFILE)
    # 性能分析模式: O3优化 + 调试信息 + 帧指针 (perf需要)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -fno-omit-frame-pointer -march=native -mtune=native -Wno-unused-result -lpthread")
    # set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler -rdc=true -arch=compute_89 -code=sm_89 -Xcompiler -std=c++17 -g -G -O3")
    message(STATUS "PROFILE mode enabled: Using O3 optimization with debug symbols for profiling")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wno-unused-result -lpthread")
    # set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -allow-unsupported-compiler -rdc=true -arch=compute_89 -code=sm_89 -Xcompiler -std=c++17 -g -G -O3")
endif()

message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")

include_directories(
    /usr/include
    src/include/
    src/
    impl/         # Phase C: 算法边界/插件迁移后统一从此目录解析算法头
    impl/common/  # Phase 2 重构：合并 impl/ 各算法 Binary/Columnar main 的共享工具
    /usr/include/jemalloc
    deps/json     # Phase A/B: nlohmann/json single header
    # ${CUDA_INCLUDE_DIRS}
)

set(PMSF_OPENMESH_ROOT ""
    CACHE PATH
    "OpenMesh 11.x prefix containing include/OpenMesh and libOpenMeshCore/Tools")
if(NOT PMSF_OPENMESH_ROOT AND DEFINED ENV{PMSF_OPENMESH_ROOT})
    set(PMSF_OPENMESH_ROOT "$ENV{PMSF_OPENMESH_ROOT}"
        CACHE PATH
        "OpenMesh 11.x prefix containing include/OpenMesh and libOpenMeshCore/Tools"
        FORCE)
endif()

if(PMSF_OPENMESH_ROOT)
    # Re-probe the complete prefix on every configure, including after switching
    # roots. Empty variables would prevent find_path/find_library from searching.
    unset(PMSF_OPENMESH_INCLUDE_DIR CACHE)
    unset(PMSF_OPENMESH_CORE_LIBRARY CACHE)
    unset(PMSF_OPENMESH_TOOLS_LIBRARY CACHE)
    find_path(PMSF_OPENMESH_INCLUDE_DIR
        NAMES OpenMesh/Core/Geometry/VectorT.hh
        PATHS
            "${PMSF_OPENMESH_ROOT}/include"
            "${PMSF_OPENMESH_ROOT}"
        NO_DEFAULT_PATH)
    find_library(PMSF_OPENMESH_CORE_LIBRARY
        NAMES OpenMeshCore
        PATHS
            "${PMSF_OPENMESH_ROOT}/lib"
            "${PMSF_OPENMESH_ROOT}/lib64"
        NO_DEFAULT_PATH)
    find_library(PMSF_OPENMESH_TOOLS_LIBRARY
        NAMES OpenMeshTools
        PATHS
            "${PMSF_OPENMESH_ROOT}/lib"
            "${PMSF_OPENMESH_ROOT}/lib64"
        NO_DEFAULT_PATH)
    if(PMSF_OPENMESH_INCLUDE_DIR AND
       PMSF_OPENMESH_CORE_LIBRARY AND
       PMSF_OPENMESH_TOOLS_LIBRARY)
        add_library(PMSF::OpenMeshCore UNKNOWN IMPORTED)
        set_target_properties(PMSF::OpenMeshCore PROPERTIES
            IMPORTED_LOCATION "${PMSF_OPENMESH_CORE_LIBRARY}")
        add_library(PMSF::OpenMeshTools UNKNOWN IMPORTED)
        set_target_properties(PMSF::OpenMeshTools PROPERTIES
            IMPORTED_LOCATION "${PMSF_OPENMESH_TOOLS_LIBRARY}")
        set(PMSF_OPENMESH_AVAILABLE ON)
        list(APPEND CMAKE_BUILD_RPATH
            "${PMSF_OPENMESH_ROOT}/lib"
            "${PMSF_OPENMESH_ROOT}/lib64")
        list(APPEND CMAKE_INSTALL_RPATH
            "${PMSF_OPENMESH_ROOT}/lib"
            "${PMSF_OPENMESH_ROOT}/lib64")
        message(STATUS "Using OpenMesh dependency from: ${PMSF_OPENMESH_ROOT}")
    else()
        message(FATAL_ERROR
            "PMSF_OPENMESH_ROOT was set but OpenMesh 11 headers/libraries "
            "were not found; specify a complete installed OpenMesh prefix")
    endif()
endif()

function(collect_source_files DIRECTORY_NAME)
    file(GLOB_RECURSE ${DIRECTORY_NAME}_SOURCES
        "src/${DIRECTORY_NAME}/*.h"
        "src/${DIRECTORY_NAME}/*.c"
        "src/${DIRECTORY_NAME}/*.cpp"
    )
    set(${DIRECTORY_NAME}_SOURCES ${${DIRECTORY_NAME}_SOURCES} PARENT_SCOPE)
endfunction()

add_subdirectory(deps/MCGAL)

# MCGAL vendors a deliberately small OpenMesh-compatible tree under
# `deps/MCGAL/include/openmesh`, while some of its legacy translation units
# retain the upstream `OpenMesh/Core/...` include spelling. Create the
# compatibility prefix in the build tree so those includes resolve without
# changing the vendored dependency sources.
set(PMSF_MCGAL_OPENMESH_COMPAT
    "${CMAKE_BINARY_DIR}/mcgal_openmesh_compat")
file(MAKE_DIRECTORY "${PMSF_MCGAL_OPENMESH_COMPAT}/OpenMesh")
if(NOT EXISTS "${PMSF_MCGAL_OPENMESH_COMPAT}/OpenMesh/Core")
    file(CREATE_LINK
         "${CMAKE_SOURCE_DIR}/deps/MCGAL/include/openmesh"
         "${PMSF_MCGAL_OPENMESH_COMPAT}/OpenMesh/Core"
         SYMBOLIC)
endif()
target_include_directories(MCGAL_static PUBLIC
    "${PMSF_MCGAL_OPENMESH_COMPAT}")
if(PMSF_OPENMESH_AVAILABLE)
    target_include_directories(MCGAL_static BEFORE PUBLIC
        "${PMSF_MCGAL_OPENMESH_COMPAT}"
        "${PMSF_OPENMESH_INCLUDE_DIR}")
    set_property(TARGET MCGAL_static PROPERTY LINK_LIBRARIES
        "PMSF::OpenMeshCore;PMSF::OpenMeshTools")
    set_property(TARGET MCGAL_static PROPERTY INTERFACE_LINK_LIBRARIES
        "PMSF::OpenMeshCore;PMSF::OpenMeshTools")
    target_include_directories(MCGAL_shared BEFORE PUBLIC
        "${PMSF_MCGAL_OPENMESH_COMPAT}"
        "${PMSF_OPENMESH_INCLUDE_DIR}")
    set_property(TARGET MCGAL_shared PROPERTY LINK_LIBRARIES
        "PMSF::OpenMeshCore;PMSF::OpenMeshTools")
    set_property(TARGET MCGAL_shared PROPERTY INTERFACE_LINK_LIBRARIES
        "PMSF::OpenMeshCore;PMSF::OpenMeshTools")
endif()

# Phase 3 重构 DSL：pmsf_add_algo_pair / pmsf_add_tool
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PMSFAlgo.cmake)

# 自动收集所有子目录
collect_source_files(common)
collect_source_files(compress)
collect_source_files(core)
collect_source_files(decompress)
collect_source_files(elimination)
collect_source_files(handler)
collect_source_files(header)
collect_source_files(operator)
collect_source_files(options)
collect_source_files(postprocess)
collect_source_files(preprocess)
collect_source_files(reconstruct)
collect_source_files(segmentation)
collect_source_files(selection)
collect_source_files(scheme)   # Phase S0（R13）：存储方案注入接口与注册表
collect_source_files(conquest) # P13.3（v31）：前沿式顶点删减引擎（算法无关）
collect_source_files(rangecoder)
collect_source_files(boundary)
# Phase 0 (connectivity-bitstream) — BitWriter/BitReader + ConnectivityBlobContainer
# 纯工具类，没有依赖其他算子/算法，可被全部 target 无条件链入。
collect_source_files(utils)
# Phase 10 重构 —— 新增 5 个算法 hook 目录。纯加法；不接入任何现有算法路径
# 之前不会被使用，保证 bpv/时延/误差零回归。
collect_source_files(metric)
collect_source_files(predict)
collect_source_files(entropy)
collect_source_files(codebook)
collect_source_files(feature)
# Phase 1（2026-09-15）：算法契约层 —— ISimplificationAlgorithm / AlgorithmRegistry。
collect_source_files(algorithm)
# Phase 6（2026-09-15）：统一随机访问 API —— Query{roi, max_lod}。
collect_source_files(query)

# Phase A/B cleanup: per-algorithm standalone binaries have been removed in favor of
# the unified config-driven PMSFCompress/PMSFDecompress. The old QEM/ICE/POMAR source
# globs below are no longer referenced by any target and are removed to reduce clutter.

# 收集框架核心文件
file(GLOB_RECURSE FRAMEWORK
    "src/PMSF.h"
    "src/Options.h"
    "src/PMSF.cpp"
)

find_package(ZLIB REQUIRED)

# ============ brotli：容器级可选压缩后端（可选）============
# 只被 `_compact.bin` 的容器压缩器用到（`PMSF_COMPACT_CODEC=brotli`，默认关闭）。
# 找不到就静默回落 zlib —— 不定义 PMSF_HAS_BROTLI 即可，默认路径字节不变。
find_library(BROTLIENC_LIBRARY brotlienc)
find_library(BROTLIDEC_LIBRARY brotlidec)
find_library(BROTLICOMMON_LIBRARY brotlicommon)
find_path(BROTLI_INCLUDE_DIR brotli/encode.h)
if(BROTLIENC_LIBRARY AND BROTLIDEC_LIBRARY AND BROTLI_INCLUDE_DIR)
    message(STATUS "Found brotli: ${BROTLIENC_LIBRARY} ${BROTLIDEC_LIBRARY}")
    set(BROTLI_LIBRARIES ${BROTLIENC_LIBRARY} ${BROTLIDEC_LIBRARY})
    if(BROTLICOMMON_LIBRARY)
        list(APPEND BROTLI_LIBRARIES ${BROTLICOMMON_LIBRARY})
    endif()
else()
    message(STATUS "brotli not found, PMSF_COMPACT_CODEC=brotli will fall back to zlib")
    set(BROTLI_LIBRARIES "")
endif()

# ============ Apache Arrow / Parquet (required, C++ >= 19) ============
# parquet::arrow::OpenFile's Result overload was introduced in Arrow 19.
set(PMSF_ARROW_MIN_VERSION "19.0.0")
set(PARQUET_SUPPORT OFF)
# These used to be find_file cache entries. Never reuse a previous wheel's ABI
# or paths after changing the Python installation or upgrading pyarrow.
foreach(_pmsf_pyarrow_library IN ITEMS PYARROW_ARROW_LIBRARY PYARROW_PARQUET_LIBRARY)
    unset(${_pmsf_pyarrow_library} CACHE)
    unset(${_pmsf_pyarrow_library})
endforeach()

find_package(Arrow QUIET CONFIG)
find_package(Parquet QUIET CONFIG)
if(Arrow_FOUND OR Parquet_FOUND OR
   TARGET Arrow::arrow_shared OR TARGET Parquet::parquet_shared)
    if(NOT ARROW_VERSION)
        set(ARROW_VERSION "${Arrow_VERSION}")
    endif()
    if(NOT PARQUET_VERSION)
        set(PARQUET_VERSION "${Parquet_VERSION}")
    endif()
    # Imported system targets cannot be safely replaced by half a wheel pair.
    # Reject old/incomplete packages before compiling any PMSF source.
    if(NOT Arrow_FOUND OR NOT Parquet_FOUND OR
       NOT TARGET Arrow::arrow_shared OR NOT TARGET Parquet::parquet_shared OR
       NOT ARROW_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+" OR
       NOT PARQUET_VERSION MATCHES "^[0-9]+\\.[0-9]+\\.[0-9]+" OR
       ARROW_VERSION VERSION_LESS PMSF_ARROW_MIN_VERSION OR
       PARQUET_VERSION VERSION_LESS PMSF_ARROW_MIN_VERSION OR
       NOT ARROW_VERSION VERSION_EQUAL PARQUET_VERSION)
        message(FATAL_ERROR
            "PMSF requires a complete, matching Arrow/Parquet C++ shared-library "
            "pair >= ${PMSF_ARROW_MIN_VERSION} (Result-returning OpenFile API). "
            "Detected system Arrow='${ARROW_VERSION}' (found=${Arrow_FOUND}), "
            "Parquet='${PARQUET_VERSION}' (found=${Parquet_FOUND}); refusing "
            "old, mismatched or incomplete packages, and refusing to mix them "
            "with pyarrow. Install a matching system pair, or select a complete "
            "pyarrow installation using -DCMAKE_DISABLE_FIND_PACKAGE_Arrow=ON "
            "-DCMAKE_DISABLE_FIND_PACKAGE_Parquet=ON "
            "-DPython3_EXECUTABLE=/path/to/python.")
    endif()
    message(STATUS "Using system Arrow/Parquet C++ ${ARROW_VERSION}")
    set(PARQUET_SUPPORT ON)
else()
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(NOT Python3_Interpreter_FOUND)
        message(FATAL_ERROR
            "PMSF requires Arrow/Parquet C++ >= ${PMSF_ARROW_MIN_VERSION}. "
            "No system pair or Python3 interpreter for the pyarrow fallback "
            "was found; set Python3_EXECUTABLE or install the C++ packages.")
    endif()

    # Query one interpreter for C++ version, headers and both libraries. Use
    # real versioned files, not wheel __version__ or user-created .so symlinks.
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -B -c [=[
import pathlib
import re
import sys
import pyarrow

version = pyarrow.cpp_version
match = re.match(r"^(\d+)\.(\d+)\.(\d+)", version)
if not match or int(match[1]) < 19:
    sys.exit("pyarrow C++ " + version + " is unsupported: Arrow/Parquet >= 19.0.0 "
             "is required for the Result-returning OpenFile API")
abi = int(match[1]) * 100 + int(match[2])
include = pathlib.Path(pyarrow.get_include()).resolve()
for header in ("arrow/api.h", "parquet/arrow/reader.h", "parquet/arrow/writer.h"):
    if not (include / header).is_file():
        sys.exit("pyarrow C++ headers missing: " + str(include / header))

def library(directory, name):
    if sys.platform.startswith("linux"):
        patterns = (f"lib{name}.so.{abi}", f"lib{name}.so.{abi}.*")
    elif sys.platform == "darwin":
        patterns = (f"lib{name}.{abi}.dylib", f"lib{name}.{abi}.*.dylib")
    else:
        sys.exit("pyarrow C++ fallback supports Linux/macOS shared libraries only")
    for pattern in patterns:
        for path in sorted(directory.glob(pattern)):
            if path.is_file() and not path.is_symlink():
                return path.resolve()

directories = [pathlib.Path(p).resolve() for p in pyarrow.get_library_dirs()]
for directory in directories:
    arrow = library(directory, "arrow")
    parquet = library(directory, "parquet")
    if arrow and parquet and arrow.parent == parquet.parent:
        print(version)
        print(include)
        print(arrow)
        print(parquet)
        break
else:
    sys.exit(f"pyarrow C++ {version}: a complete Arrow/Parquet shared-library "
             f"pair with ABI {abi} was not found in {directories}")
]=]
        OUTPUT_VARIABLE _pmsf_pyarrow_probe
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_VARIABLE _pmsf_pyarrow_error
        RESULT_VARIABLE PYARROW_PROBE_RC
        TIMEOUT 60)
    if(NOT "${PYARROW_PROBE_RC}" STREQUAL "0")
        message(FATAL_ERROR
            "PMSF requires a complete Arrow/Parquet C++ pair >= "
            "${PMSF_ARROW_MIN_VERSION}. No system pair was found and the "
            "pyarrow probe (${Python3_EXECUTABLE}) failed: ${_pmsf_pyarrow_error}")
    endif()
    string(REPLACE "\r\n" "\n" _pmsf_pyarrow_probe "${_pmsf_pyarrow_probe}")
    string(REPLACE "\n" ";" _pmsf_pyarrow_fields "${_pmsf_pyarrow_probe}")
    list(LENGTH _pmsf_pyarrow_fields _pmsf_pyarrow_field_count)
    if(NOT _pmsf_pyarrow_field_count EQUAL 4)
        message(FATAL_ERROR "Invalid pyarrow dependency probe output: ${_pmsf_pyarrow_probe}")
    endif()
    list(GET _pmsf_pyarrow_fields 0 ARROW_VERSION)
    set(PARQUET_VERSION "${ARROW_VERSION}")
    list(GET _pmsf_pyarrow_fields 1 PYARROW_INCLUDE_DIR)
    list(GET _pmsf_pyarrow_fields 2 PYARROW_ARROW_LIBRARY)
    list(GET _pmsf_pyarrow_fields 3 PYARROW_PARQUET_LIBRARY)

    add_library(Arrow::arrow_shared SHARED IMPORTED)
    set_target_properties(Arrow::arrow_shared PROPERTIES
        IMPORTED_LOCATION "${PYARROW_ARROW_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PYARROW_INCLUDE_DIR}")
    add_library(Parquet::parquet_shared SHARED IMPORTED)
    set_target_properties(Parquet::parquet_shared PROPERTIES
        IMPORTED_LOCATION "${PYARROW_PARQUET_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PYARROW_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES Arrow::arrow_shared)
    get_filename_component(PYARROW_RUNTIME_DIR "${PYARROW_ARROW_LIBRARY}" DIRECTORY)
    list(APPEND CMAKE_BUILD_RPATH "${PYARROW_RUNTIME_DIR}")
    list(APPEND CMAKE_INSTALL_RPATH "${PYARROW_RUNTIME_DIR}")
    set(PARQUET_SUPPORT ON)
    message(STATUS
        "Using pyarrow C++ ${ARROW_VERSION}: ${PYARROW_RUNTIME_DIR} "
        "(Python: ${Python3_EXECUTABLE}, headers: ${PYARROW_INCLUDE_DIR})")
endif()
find_library(ZSTD_LIBRARY zstd)
if(ZSTD_LIBRARY)
    message(STATUS "Found zstd: ${ZSTD_LIBRARY}")
endif()

# ============ Protobuf：已整体退役（2026-09-17）============
#
# 移除原因：这套东西**整套都是死的**，且留着一个会在被构建时立刻失败的 target：
#   - `proto/pmsf_mesh.proto` 早已不存在，而下面的 add_custom_command 仍以它作
#     输入 —— 谁 build 了 `proto_gen` 谁就报错；
#   - `ProtobufSerializeOperator` / `ProtobufDeserializeOperator` 被下面的
#     FILTER 排除出所有目标，从不编译；
#   - `tools/Protobuf{CompressServer,DecompressClient}.cpp` 不在任何 target 里；
#   - 唯一 include 这两个头的文件就是上面这几个，互为孤岛。
# 相应源文件与头文件已一并删除。若将来要恢复 Protobuf 互操作，请按
# cmake/PMSFAlgo.cmake 的 DSL 重新接入，而不是复活这套。
#
# 注意：下面这条 FILTER 保留 —— 见 CLAUDE.md「新文件名避开 Parquet/Protobuf 子串」
# 那条 gotcha，它同时是给未来新文件的护栏。

# 从ALL_SOURCES中排除Parquet相关源文件（避免非对应目标编译报错）
list(FILTER compress_SOURCES EXCLUDE REGEX ".*Parquet.*")
list(FILTER decompress_SOURCES EXCLUDE REGEX ".*Parquet.*")
list(FILTER compress_SOURCES EXCLUDE REGEX ".*Protobuf.*")
list(FILTER decompress_SOURCES EXCLUDE REGEX ".*Protobuf.*")

# 重新合并ALL_SOURCES（排除Parquet文件后）
set(ALL_SOURCES
    ${FRAMEWORK}
    ${common_SOURCES}
    ${compress_SOURCES}
    ${core_SOURCES}
    ${decompress_SOURCES}
    ${elimination_SOURCES}
    ${postprocess_SOURCES}
    ${preprocess_SOURCES}
    ${handler_SOURCES}
    ${header_SOURCES}
    ${operator_SOURCES}
    ${options_SOURCES}
    ${reconstruct_SOURCES}
    ${segmentation_SOURCES}
    ${selection_SOURCES}
    ${rangecoder_SOURCES}
    ${boundary_SOURCES}
    ${utils_SOURCES}
    ${scheme_SOURCES}   # Phase S0（R13）：存储方案注入接口与注册表
    # P13.3（v31）：前沿式顶点删减引擎 + 解码镜像。**必须在这里。**
    # 引擎实现此前只在 `ConquestLayerRoundTripProbe` 里被显式列过源文件，而
    # `impl/Conquest/` 下的编码端薄壳会被 `ALGO_MIGRATED_SOURCES` 通配捡进
    # PMSFCompress —— 两者一撞就是
    # `undefined reference to FrontierConquestEngine::decimateLayer`。
    # （该失败此前一直是**默认构建红的**，只是被建脚本 | tail 掩盖了退出码。）
    # 该引擎在既有默认路径上无人调用，链进来不改变任何产物字节（回归门禁为证）。
    ${conquest_SOURCES}
    # Phase 10 hook 目录（加法式；空目录也不会破坏构建）
    ${metric_SOURCES}
    ${predict_SOURCES}
    ${entropy_SOURCES}
    ${codebook_SOURCES}
    ${feature_SOURCES}
    # Phase 1 算法契约层（ISimplificationAlgorithm / AlgorithmRegistry）
    ${algorithm_SOURCES}
    # Phase 6 随机访问 API（Query）
    ${query_SOURCES}
)

# ColumnarDeserializeOperator 实现已迁移到 impl/common，但仍被 ALL_SOURCES
# 驱动的测试与通用工具依赖，因此显式追加到 ALL_SOURCES。
list(APPEND ALL_SOURCES "impl/common/ColumnarDeserializeOperator.cpp")

# PHASE60 物品 3：分区（layout-32）产物的后端无关解码镜像。两个消费者：
# impl/common/ColumnarDeserializeOperator.cpp（列式/Unified）与
# src/decompress/OptimizedBasicDeserializeOperator.cpp（二进制）。
list(APPEND ALL_SOURCES "impl/common/SegmentedRangeCodedDecode.cpp")

# Phase S0/S1（R13）：示例存储方案。方案实现按设计文档 §6 放在 impl/，
# src/ 只认识接口与注册表 —— 因此这里显式追加，而不是让 src/ 反向依赖。
list(APPEND ALL_SOURCES "impl/common/StorageSchemeDemo.cpp")

# Parquet相关源文件（单独管理）
set(PARQUET_SOURCES
    "src/compress/ParquetSerializeOperator.cpp"
    "src/compress/ParquetSerializeOperator.h"
    "src/decompress/ParquetDeserializeOperator.cpp"
    "src/decompress/ParquetDeserializeOperator.h"
)

# Parquet V2 结构化列式存储相关源文件
set(COLUMNAR_SOURCES
    "src/compress/ColumnarSerializeOperator.cpp"
    "src/compress/ColumnarSerializeOperator.h"
    "src/compress/ColumnarBlobSerializeOperator.cpp"
    "src/compress/ColumnarBlobSerializeOperator.h"
    "impl/POMAR/compress/POMARColumnarSerializeOperator.cpp"
    "impl/POMAR/compress/POMARColumnarSerializeOperator.h"
    "src/decompress/ColumnarDeserializeOperator.h"
    "src/decompress/ColumnarBlobDeserializeOperator.cpp"
    "src/decompress/ColumnarBlobDeserializeOperator.h"
    "impl/POMAR/decompress/POMARColumnarDeserializeOperator.cpp"
    "impl/POMAR/decompress/POMARColumnarDeserializeOperator.h"
    # PaperStream v1 (M2b pilot): single-file `_paper.parquet` writer/reader.
    # The *Parquet* suffix keeps the huge Arrow/Parquet translation unit out of
    # non-columnar ALL_SOURCES targets (see the CLAUDE.md filename gotcha).
    "src/compress/PaperSerializeOperator.h"
    "src/compress/PaperSerializeOperatorParquet.cpp"
    "src/decompress/PaperDeserializeOperator.h"
    "src/decompress/PaperDeserializeOperatorParquet.cpp"
    "src/decompress/PaperDecompressFactoryRegistrationParquet.cpp"
    "src/storage/PaperStreamWriter.cpp"
    "src/storage/PaperStreamWriter.h"
    "src/storage/PaperStreamReader.cpp"
    "src/storage/PaperStreamReader.h"
    "src/storage/PaperStreamWriterV2.cpp"
    "src/storage/PaperStreamWriterV2.h"
    "src/storage/PaperStreamReaderV2.cpp"
    "src/storage/PaperStreamReaderV2.h"
    # M5-5 opaque-payload artifact glue: used by the VQMMC paper branch
    # (PMSFCompress/PMSFDecompress link COLUMNAR_SOURCES).
    "src/storage/PaperStreamOpaquePayloadV2.cpp"
    "src/storage/PaperStreamOpaquePayloadV2.h"
)

# Phase 11: Parquet V3 动态 side-channel 列扩展
# V3 与 V2 并列，算法按需选择；本阶段只用于 test_unified smoke 验证
set(UNIFIED_SOURCES
    "src/compress/UnifiedSerializeOperator.cpp"
    "src/compress/UnifiedSerializeOperator.h"
    "src/compress/UnifiedSchema.h"
)

# v3-refactor 分支：Parquet V3 多表 writer/reader（继承 V2 schema + 追加
# sidechannel / blob 两张可选表）。9 算法 + default 的 Unified{Compress,Decompress}
# 变体通过 `pmsf_add_algo_pair(... )` 的 V3 扩展一次性注册，需要把这组 sources
# 与 V2 sources 一起链到 target 里。
set(UNIFIED_MULTI_TABLE_SOURCES
    "src/compress/UnifiedSerializeOperator.cpp"
    "src/compress/UnifiedSerializeOperator.h"
    "src/compress/UnifiedSchema.h"
    "src/compress/UnifiedMultiTableSerializeOperator.cpp"
    "src/compress/UnifiedMultiTableSerializeOperator.h"
    "src/decompress/UnifiedMultiTableDeserializeOperator.cpp"
    "src/decompress/UnifiedMultiTableDeserializeOperator.h"
    # 阶段 3a：POMAR 专属 V3 serializer（继承 V2 专属，额外写 _v3_blob.parquet）
    "impl/POMAR/compress/POMARUnifiedSerializeOperator.cpp"
    "impl/POMAR/compress/POMARUnifiedSerializeOperator.h"
)

# Arrow IPC / ORC smoke storage backends. These are intentionally kept out of
# the auto-collected src/compress and src/decompress source sets so non-Arrow
# targets do not need to link libarrow.
set(COLUMNAR_BLOB_SOURCES
    "src/storage/ColumnarBlobOperators.cpp"
    "src/storage/ColumnarBlobOperators.h"
)

# ============================================================================
# Tools (tools/*.cpp) — 非算法的框架工具可执行文件
# 阶段 D：算法专属的 legacy wrapper 工具已移除，统一由配置驱动的
# PMSFCompress / PMSFDecompress 分发。
# ============================================================================
# 注意：pmsf_add_tool 依赖 PMSF_ENTRY_SOURCES / PMSF_VQMMC_OPERATOR_SOURCES
# （ALL_SOURCES 里的 impl/common/ColumnarDeserializeOperator.cpp 需要它们的符号），
# 因此调用点必须晚于下方这两个变量的定义——见文件末尾的 SizeTest 注册。

# ============================================================================
# Algorithms — 9 篇论文对应的 Binary + Columnar 入口（已弃用）
# ============================================================================
# Phase A/B: 这些独立的算法可执行文件已被配置驱动的 PMSFCompress/PMSFDecompress
# 取代。所有算法入口现在统一通过 impl/<algo>/PMSFEntry.cpp 注册到
# AlgorithmRegistry，由 tools/PMSFCompress.cpp 与 tools/PMSFDecompress.cpp 根据
# config.json 中的 "algorithm" 字段分发。保留注释以便查阅历史 target 列表。
# pmsf_add_algo_pair(QEM)
# pmsf_add_algo_pair(PPMC)
# pmsf_add_algo_pair(APTM)
# pmsf_add_algo_pair(PCATM)
# pmsf_add_algo_pair(PCLTTM)
# pmsf_add_algo_pair(RD)
# pmsf_add_algo_pair(MMSE
#     COMPRESS_EXTRA    "impl/MMSE/MMSEPostProcessor.cpp"
#     DECOMPRESS_EXTRA  "impl/MMSE/MMSEPostProcessor.cpp")
# pmsf_add_algo_pair(VQMMC
#     COMPRESS_EXTRA    "impl/VQMMC/VQMMCPostProcessor.cpp"
#                       "impl/VQMMC/VQMMCSymbolCollectOperator.cpp"
#                       "impl/VQMMC/VectorQuantizer.cpp"
#                       "impl/VQMMC/VQMMCColumnarSerializeOperator.cpp"
#                       "impl/VQMMC/VQMMCStrictSymbolCollectOperator.cpp"
#     DECOMPRESS_EXTRA  "impl/VQMMC/VQMMCPostProcessor.cpp"
#                       "impl/VQMMC/VQMMCSymbolReadOperator.cpp"
#                       "impl/VQMMC/VQMMCReconstructOperator.cpp"
#                       "impl/VQMMC/VQMMCDeserializeOperator.cpp"
#                       "impl/VQMMC/VQMMCColumnarDeserializeOperator.cpp"
#                       "impl/VQMMC/VectorQuantizer.cpp"
#                       "impl/VQMMC/VQMMCStrictSymbolReadOperator.cpp")
# pmsf_add_algo_pair(POMAR)
# pmsf_add_algo_pair(POMAR
#     BINARY_ONLY
#     COMPRESS_SUFFIX   V2Compress
#     DECOMPRESS_SUFFIX V2Decompress)
# pmsf_add_algo_pair(PCLTTMValence
#     DIR impl/PCLTTM
#     EXTRA_INCLUDES impl/PCLTTM)
# pmsf_add_algo_pair(HelloHook)

# ============================================================================
# Phase A/B: 配置驱动的统一压缩/解压入口
# 所有算法-specific 逻辑直接编译进 tools/PMSFCompress.cpp 与 PMSFDecompress.cpp，
# 不再单独维护 adapter 文件。
# ============================================================================
# Phase A/B: 各算法入口注册文件。每个文件在自己的编译单元里实现并注册 IAlgorithmEntry，
# 避免把全部算法-specific 头文件塞进同一个 translation unit 导致 operator 类重定义。
file(GLOB_RECURSE ALGO_MIGRATED_SOURCES
    "impl/*/compress/*.cpp"
    "impl/*/decompress/*.cpp"
    "impl/*/elimination/*.cpp"
    "impl/*/reconstruct/*.cpp")

# Phase 1：算法插件 —— 约定 `impl/<ALGO>/<Algo>Algorithm.cpp`。
# 一个算法 = 一个目录 + 一个 cpp；**删目录即卸载算法**，不再需要维护手工清单。
# （旧形态 `impl/<ALGO>/PMSFEntry.cpp` 由下面的显式列表继续保留，Phase 4 后移除。）
file(GLOB_RECURSE PMSF_PLUGIN_SOURCES "impl/*/*Algorithm.cpp")

set(PMSF_ENTRY_SOURCES
    ${PMSF_PLUGIN_SOURCES}
    "impl/common/DecompressFactoryRegistration.cpp"
    # 只为副作用存在：把协议覆盖的实现注册进 src/ 的接缝。见文件头注释 ——
    # 头文件静态初始化只在被包含时存在，摘掉最后一个包含者会静默改变产物。
    "impl/common/RetriOverrideRegistration.cpp"
    "impl/common/CollectorLayoutPlugin.cpp"
    "impl/common/CanonicalSymbolStreamRegistry.cpp"
    "impl/common/PMSFRetriangulation.cpp"
    "impl/common/CollectorSymbolsTableBuilder.cpp"
    "impl/POMAR/DecompressFactoryRegistration.cpp"
    # ⚠️ **`impl/<ALGO>/DecompressFactoryRegistration.cpp` 落在所有 glob 之外**
    # （`impl/*/decompress/*.cpp` 不匹配根目录文件，`impl/*/*Algorithm.cpp` 只收
    # `*Algorithm.cpp`）⇒ 每个算法的登记文件都必须像上面 POMAR 那样**显式列**。
    # 漏掉的后果是**静默的**：解压时该算法名查不到三件套 → 回落到框架 Default
    # 三件套（reader 成了 `OptimizedSymbolReadOperator`）→ 读到本算法的 blob 就
    # 在**别人的代码里**段错。实测就是这样栽的：栈顶是
    # `OptimizedSymbolReadOperator::importFromRangeCodedBuffer`，
    # 离 conquest 很远，排查绕了很远的路。
    "impl/Conquest/DecompressFactoryRegistration.cpp"
    "impl/PPMC/bindings/PPMCBindings.cpp"
    # P1（LoC 重构 2026-09-23）：RD patch 簇整体移到 impl/common/patch/。
    # ⚠️ 移出后**不再被 ALGO_MIGRATED_SOURCES（impl/*/compress|decompress/*.cpp）
    # 匹配** ⇒ 五个 .cpp 必须逐个显式列出，漏一个就是链接期 undefined reference
    # （或运行期静默回落），与之前 DecompressFactoryRegistration 落在 glob 外同款。
    "impl/common/patch/RDPatchPipeline.cpp"
    "impl/common/patch/RDPatchDecoderConquest.cpp"
    "impl/common/patch/RDPatchGateQueue.cpp"
    "impl/common/patch/RDGateOnlySymbolCollectOperator.cpp"
    "impl/common/patch/RDGateOnlySymbolReadOperator.cpp"
    # P1-b：PCLTTMStrict 簇移到 impl/common/strict/（同 §7.1 的 glob 陷阱，
    # 移出 impl/*/compress|decompress/ 后不再被 ALGO_MIGRATED_SOURCES 匹配）。
    "impl/common/strict/PCLTTMStrictSymbolCollectOperator.cpp"
    "impl/common/strict/PCLTTMStrictSymbolReadOperator.cpp"
    # P1-c：PCATMStrict 簇移到 impl/common/strict/（同 §7.1 的 glob 陷阱）。
    "impl/common/strict/PCATMStrictPMSFSymbolCollectOperator.cpp"
    "impl/common/strict/PCATMStrictPMSFSymbolReadOperator.cpp"
    ${ALGO_MIGRATED_SOURCES}
)

# The POMAR plugin now maps its native streams into the shared PaperStream v2
# writer/reader. Every target that compiles the plugin operators must link the
# same storage implementation, including hook/Arrow/ORC smoke targets that do
# not otherwise use COLUMNAR_SOURCES. CMake deduplicates sources also listed by
# the full columnar targets.
if(PARQUET_SUPPORT)
    list(APPEND PMSF_ENTRY_SOURCES
        "src/storage/PaperStreamWriterV2.cpp"
        "src/storage/PaperStreamReaderV2.cpp"
        "src/storage/PaperStreamReader.cpp")
endif()

# VQMMC 算子实现文件（位于 impl/VQMMC 根目录，未纳入 impl/*/compress/ 迁移目录，
# 但仍需随统一入口一起编译）。
set(PMSF_VQMMC_OPERATOR_SOURCES
    "impl/VQMMC/VQMMCPostProcessor.cpp"
    "impl/VQMMC/VQMMCSymbolCollectOperator.cpp"
    "impl/VQMMC/VQMMCSymbolReadOperator.cpp"
    "impl/VQMMC/VQMMCReconstructOperator.cpp"
    "impl/VQMMC/VQMMCDeserializeOperator.cpp"
    "impl/VQMMC/VQMMCColumnarSerializeOperator.cpp"
    "impl/VQMMC/VQMMCColumnarDeserializeOperator.cpp"
    "impl/VQMMC/VQMMCStrictSymbolCollectOperator.cpp"
    "impl/VQMMC/VQMMCStrictSymbolReadOperator.cpp"
    "impl/VQMMC/VQMMCPaperDeserializeOperator.cpp"
    "impl/VQMMC/VQMMCPaperRegistration.cpp"
    "impl/VQMMC/VectorQuantizer.cpp"
)

foreach(_t IN ITEMS PMSFCompress PMSFDecompress)
    add_executable(${_t}
        tools/${_t}.cpp
        "src/config/PMSFConfigParser.cpp"
        "src/hook/HookRegistry.cpp"
        "impl/MMSE/MMSEPostProcessor.cpp"
        ${PMSF_ENTRY_SOURCES}
        ${PMSF_VQMMC_OPERATOR_SOURCES}
        ${COLUMNAR_SOURCES}
        ${UNIFIED_MULTI_TABLE_SOURCES}
        ${ALL_SOURCES})
    target_include_directories(${_t} PUBLIC "src" "impl" "impl/PPMC" "impl/VQMMC" "impl/POMAR" "impl/QEM" "impl/RD" "impl/MMSE" "impl/APTM" "impl/PCATM" "impl/PCLTTM")
    target_compile_definitions(${_t} PRIVATE COLUMNAR_SUPPORT PMSF_DEFAULT_BACKEND_COLUMNAR)
    if(ZSTD_LIBRARY)
        target_compile_definitions(${_t} PRIVATE PMSF_HAS_ZSTD)
    endif()
    if(BROTLI_LIBRARIES)
        target_compile_definitions(${_t} PRIVATE PMSF_HAS_BROTLI)
        target_include_directories(${_t} PRIVATE "${BROTLI_INCLUDE_DIR}")
    endif()
    target_link_libraries(${_t}
        MCGAL_static ZLIB::ZLIB
        Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl
        ${ZSTD_LIBRARY} ${BROTLI_LIBRARIES})
    if(DEBUG)
        target_compile_definitions(${_t} PRIVATE DEBUG)
    endif()
endforeach()

# ============================================================================
# Segmented PPMC owned-boundary tools (HEAD-port 2026-10)
# ----------------------------------------------------------------------------
# Two-phase round schedule: protect partition boundaries for N rounds, then
# free them for M rounds. Open boundary faces are encoded as NEGATIVE
# face_degrees (abs = path length); the decoder rebuilds an open halfedge path
# and uses create_center_vertex_plus to restore the removed boundary vertex
# (OpenBoundaryPPMCReconstructOperator in the Decompress tool).
# Driver: scripts/tests/segmented_ppmc_owned_boundary_schedule_sweep.sh.
# Restored from commit ac9a973 (= e5af44c^). Operators moved to impl/<ALGO>/ in
# the Phase-A/B refactor, so reuse the full PMSFCompress/PMSFDecompress source
# set (PMSF_ENTRY_SOURCES brings in impl/PPMC + impl/Segmentation operators).
# ============================================================================
foreach(_t IN ITEMS SegmentedPPMCColumnarOwnedBoundaryCompress
                    SegmentedPPMCColumnarOwnedBoundaryDecompress)
    add_executable(${_t}
        tools/${_t}.cpp
        "src/config/PMSFConfigParser.cpp"
        "src/hook/HookRegistry.cpp"
        "impl/MMSE/MMSEPostProcessor.cpp"
        ${PMSF_ENTRY_SOURCES}
        ${PMSF_VQMMC_OPERATOR_SOURCES}
        ${COLUMNAR_SOURCES}
        ${UNIFIED_MULTI_TABLE_SOURCES}
        ${ALL_SOURCES})
    target_include_directories(${_t} PUBLIC "src" "impl" "impl/PPMC" "impl/VQMMC" "impl/POMAR" "impl/QEM" "impl/RD" "impl/MMSE" "impl/APTM" "impl/PCATM" "impl/PCLTTM")
    target_compile_definitions(${_t} PRIVATE COLUMNAR_SUPPORT PMSF_DEFAULT_BACKEND_COLUMNAR)
    if(ZSTD_LIBRARY)
        target_compile_definitions(${_t} PRIVATE PMSF_HAS_ZSTD)
    endif()
    if(BROTLI_LIBRARIES)
        target_compile_definitions(${_t} PRIVATE PMSF_HAS_BROTLI)
        target_include_directories(${_t} PRIVATE "${BROTLI_INCLUDE_DIR}")
    endif()
    target_link_libraries(${_t}
        MCGAL_static ZLIB::ZLIB
        Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl
        ${ZSTD_LIBRARY} ${BROTLI_LIBRARIES})
    if(DEBUG)
        target_compile_definitions(${_t} PRIVATE DEBUG)
    endif()
endforeach()

# ============================================================================
# AdConquestVerify —— P13.1 的**独立验证器**。
#
# 边界（与 RDPatchBpvMeasure 同款）：只把 MCGAL 上的 AD valence-conquest 编码侧
# 跑起来、导出**连通性符号序列**，交给 tools/mepp2_reference/oracle_diff.py 与
# MEPP2 oracle 对拍。**不注册 AlgoPreset、不写产物、不进任何门禁矩阵**，
# 对 PMSFCompress/PMSFDecompress 的产物字节零影响。
#
# 依赖最小：只要 MCGAL_static（不碰 Arrow/Parquet/列存），
# 因此即使可选依赖缺失本 target 也应能构建。
# ============================================================================
add_executable(CenterVertexRoundTripProbe "tools/CenterVertexRoundTripProbe.cpp")
target_include_directories(CenterVertexRoundTripProbe PUBLIC "deps/MCGAL/include" "src")
target_sources(CenterVertexRoundTripProbe PRIVATE "src/core/MeshRuntime.cpp")
target_link_libraries(CenterVertexRoundTripProbe MCGAL_static)

add_executable(ConquestLayerRoundTripProbe
    tools/ConquestLayerRoundTripProbe.cpp
    src/conquest/ConquestDecodeLayer.cpp
    src/conquest/FrontierConquestEngine.cpp
    src/rangecoder/rangecod.c
    src/rangecoder/qsmodel.c
    src/core/MeshRuntime.cpp)
target_include_directories(ConquestLayerRoundTripProbe PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/tools
    ${CMAKE_SOURCE_DIR}/deps/MCGAL/include)
target_link_libraries(ConquestLayerRoundTripProbe PRIVATE MCGAL_static)

# P13.4 (B)②：字节层往返探针（collector → reader → 重建器）。
# 判据是纯几何+拓扑（点集/面集全等），不依赖任何自报中间量。
add_executable(ConquestCodecRoundTrip
    tools/ConquestCodecRoundTrip.cpp
    impl/Conquest/compress/ConquestSymbolCollectOperator.cpp
    impl/Conquest/decompress/ConquestSymbolReadOperator.cpp
    impl/Conquest/reconstruct/ConquestReconstructOperator.cpp
    src/conquest/ConquestDecodeLayer.cpp
    src/conquest/FrontierConquestEngine.cpp
    src/rangecoder/rangecod.c
    src/rangecoder/qsmodel.c
    src/core/MeshRuntime.cpp)
target_include_directories(ConquestCodecRoundTrip PRIVATE
    ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/impl
    ${CMAKE_SOURCE_DIR}/deps/MCGAL/include)
target_link_libraries(ConquestCodecRoundTrip PRIVATE MCGAL_static)

add_executable(ConquestEngineVerify "tools/ConquestEngineVerify.cpp")
target_include_directories(ConquestEngineVerify PUBLIC "deps/MCGAL/include" "src")
target_sources(ConquestEngineVerify PRIVATE
    "src/core/MeshRuntime.cpp"
    "src/conquest/FrontierConquestEngine.cpp"
    "src/rangecoder/rangecod.c"
    "src/rangecoder/qsmodel.c")
target_link_libraries(ConquestEngineVerify MCGAL_static)

add_executable(AdConquestVerify "tools/AdConquestVerify.cpp")
target_include_directories(AdConquestVerify PUBLIC "deps/MCGAL/include" "src")
target_sources(AdConquestVerify PRIVATE "src/core/MeshRuntime.cpp")
target_link_libraries(AdConquestVerify MCGAL_static)

# ============================================================================
# HausdorffDistance：独立工具，不依赖 MCGAL / jemalloc
# ============================================================================
add_executable(HausdorffDistance "tools/HausdorffDistance.cpp")
set_target_properties(HausdorffDistance PROPERTIES LINK_LIBRARIES "")

# ============================================================================
# SurfaceMetrics：Metro/MESH 风格面积采样距离，独立工具，不依赖 MCGAL / jemalloc
# ============================================================================
add_executable(SurfaceMetrics "tools/SurfaceMetrics.cpp")
set_target_properties(SurfaceMetrics PROPERTIES LINK_LIBRARIES "")

# ============================================================================
# RMSError：解压缩验证工具，不依赖 MCGAL / jemalloc
# ============================================================================
add_executable(RMSError "tools/RMSError.cpp")
set_target_properties(RMSError PROPERTIES LINK_LIBRARIES "")

# ============================================================================
# MeshCompare：拓扑/结构差异检查工具，不依赖 MCGAL / jemalloc
# ============================================================================
add_executable(MeshCompare "tools/MeshCompare.cpp")
set_target_properties(MeshCompare PROPERTIES LINK_LIBRARIES "")

# ============================================================================
# CompactBinInfo：读 `<base>_compact.bin` 的**自描述字段**（P30 物品4）。
# 门禁 verify_artifact_layout.sh 用它，对**所有** codec 做全字段校验 ——
# 容器改扩展后端之后，Python 只能解 codec 1，2(brotli)/3(bitctx) 的
# codecVersion/blobLayout 查不了；这里把这件事交回 C++ 解码器，
# 它本来就支持全部 codec。仅头文件依赖（PMSFCompactBinEnvelope.h 全 inline）
# ⇒ 不链 MCGAL / jemalloc。
# ============================================================================
# 依赖：bitctx 解码要用范围编码器（qsmodel.c/rangecod.c），zlib 解 codec 1。
# 两者都不是 MCGAL/jemalloc ⇒ 仍显式清空 LINK_LIBRARIES 只补这两样。
add_executable(CompactBinInfo "tools/CompactBinInfo.cpp"
               "src/rangecoder/qsmodel.c"
               "src/rangecoder/rangecod.c")
set_target_properties(CompactBinInfo PROPERTIES LINK_LIBRARIES "z")
target_include_directories(CompactBinInfo PRIVATE "${CMAKE_SOURCE_DIR}/src"
                           "${CMAKE_SOURCE_DIR}/src/rangecoder")
if(BROTLI_LIBRARIES)
    target_compile_definitions(CompactBinInfo PRIVATE PMSF_HAS_BROTLI)
    target_include_directories(CompactBinInfo PRIVATE "${BROTLI_INCLUDE_DIR}")
    target_link_libraries(CompactBinInfo PRIVATE ${BROTLI_LIBRARIES})
endif()

# v6 段目录与统一 compact envelope 的轻量回环测试。
# 只依赖 zlib/range coder。根 CMake 因缺少 Arrow 配置而失败时，
# 可用 scripts/tests/test_compact_storage.sh 单独构建/测试。
add_executable(test_compact_bin_envelope
               "tests/test_compact_bin_envelope.cpp"
               "src/rangecoder/qsmodel.c"
               "src/rangecoder/rangecod.c"
               "src/scheme/StorageSchemeRegistry.cpp")
set_target_properties(test_compact_bin_envelope PROPERTIES LINK_LIBRARIES "z")
target_include_directories(test_compact_bin_envelope PRIVATE "${CMAKE_SOURCE_DIR}/src"
                           "${CMAKE_SOURCE_DIR}/src/rangecoder")
add_executable(test_compact_bitctx "tests/test_compact_bitctx.cpp"
               "src/rangecoder/qsmodel.c" "src/rangecoder/rangecod.c")
set_target_properties(test_compact_bitctx PROPERTIES LINK_LIBRARIES "z")
target_include_directories(test_compact_bitctx PRIVATE "${CMAKE_SOURCE_DIR}/src")

add_executable(test_segmented_group_channel
               "tests/test_segmented_group_channel.cpp")
target_include_directories(test_segmented_group_channel PRIVATE
                           "${CMAKE_SOURCE_DIR}/src")

if(BROTLI_LIBRARIES)
    target_compile_definitions(test_compact_bin_envelope PRIVATE PMSF_HAS_BROTLI)
    target_include_directories(test_compact_bin_envelope PRIVATE "${BROTLI_INCLUDE_DIR}")
    target_link_libraries(test_compact_bin_envelope PRIVATE ${BROTLI_LIBRARIES})
endif()

# ============================================================================
# MeshAudit：三角网格流型审计/基础修复工具，不依赖 MCGAL / jemalloc
# ============================================================================
# MeshVertexDeviation：定位"哪些顶点被放错了位置"（HausdorffDistance 只给 max/mean/RMS
# 指不出是哪些）。同样独立，不依赖 MCGAL / jemalloc。
add_executable(MeshVertexDeviation "tools/MeshVertexDeviation.cpp")
set_target_properties(MeshVertexDeviation PROPERTIES LINK_LIBRARIES "")

add_executable(MeshAudit "tools/MeshAudit.cpp")
set_target_properties(MeshAudit PROPERTIES LINK_LIBRARIES "")

# ============================================================================
# Parquet 工具（Parquet V1 遗留 & V2 Default 工具 & ArrowParquetExample）
# ============================================================================
if(PARQUET_SUPPORT)
    find_package(CURL REQUIRED)
    find_package(PkgConfig REQUIRED)
    # Arrow/Parquet imported targets already supply include/link information.
    # Wheels do not ship arrow.pc; requiring it again defeats the C++ fallback.
    pkg_check_modules(CURL REQUIRED libcurl)

    # Parquet V1（遗留）
    foreach(_t IN ITEMS DefaultParquetCompress DefaultParquetDecompress)
        add_executable(${_t} tools/${_t}.cpp ${PARQUET_SOURCES} ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
        target_include_directories(${_t} PUBLIC "src")
        target_link_libraries(${_t}
            MCGAL_static ZLIB::ZLIB
            Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)
        if(DEBUG)
            target_compile_definitions(${_t} PRIVATE DEBUG)
        endif()
    endforeach()

    # Arrow/Parquet 综合示例
    add_executable(ArrowParquetExample example/ArrowParquetExample.cpp)
    target_link_libraries(ArrowParquetExample
        Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)

    add_executable(BinaryParquetBlob tools/BinaryParquetBlob.cpp)
    target_link_libraries(BinaryParquetBlob
        Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)

    # Parquet V2 的 Default 工具
    # 追加：DefaultPPMCLiteColumnar{Compress,Decompress} —— 在 Default v3 BFS
    # 上再加 PPMC 的 Frenet-rotated residual（codec_version=4, blob_layout=6）。
    # 追加：DefaultRDColumnar{Compress,Decompress} —— RD paper-aligned gate-conquest
    # （Phase A：与 Valence 同 byte layout，独立 blob_layout=7；Phase B 将改动
    # RDSymbolReadOperator 实现真·gate-only BFS + Frenet 逆旋转）。
    # 追加：DefaultRDGateOnlyColumnar{Compress,Decompress} —— RD Phase B（2026-05-09）。
    # v2 协议：BFS-symmetric + per-face is-polyFace bit + V_OTHER inline escape；
    # 无 edge-sym，v0-fan 由 create_center_vertex_without_init 对称重建。round-trip ✓。
    # 追加：DefaultPPMCColumnar{Compress,Decompress} —— Default 路径（OptimizedSymbolCollect
    # 替换为 PPMCSymbolCollect + PPMCEliminate + PPMCReconstruct + PPMC 4 项预测开关
    # 全开 + polygon preservation）。把 PPMC 论文 §3.2/§3.3 的优化作为 framework feature
    # 暴露给 Default 使用，blob_layout=PPMC_RC=3。
    # 追加：DefaultValencePatchOnlyColumnar{Compress,Decompress} —— Phase 1 · Step 1.5
    # （2026-05-10）方案 V 端到端 round-trip 入口。与 DefaultRDGateOnly 同体系，仅把
    # collector/reader 换成 ValencePatchOnly*，blob_layout=9 (VALENCE_PATCH_ONLY, sv=1)。
    # 追加：DefaultPCATMColorColumnar{Compress,Decompress} —— PCATM 论文 2-color 方案
    # （CP-3，2026-05-10）。Eliminate 复用 PCATMEliminateOperator（Z-shape 已落地），
    # Collect 切到 PCATMColorSymbolCollectOperator。CP-3.1 阶段 export 仍 delegate 父类
    # OptimizedCollect（blob_layout=1，与原 PCATMColumnar 字节一致）；CP-3.2+ 切到
    # BlobLayout::PCATM_COLOR=14（3 段流 valence + color + residual）。
    # Arrow IPC / ORC BLOB-backed default round-trip tools.
    # Phase 0（2026-09-15）：ALL_SOURCES 里的 impl/common/ColumnarDeserializeOperator.cpp
# 会引用各算法 impl 侧 deserializer 的 typeinfo，因此这些 target 必须一并链入
# PMSF_ENTRY_SOURCES / PMSF_VQMMC_OPERATOR_SOURCES，否则 ld 报 undefined reference。
foreach(_t IN ITEMS DefaultArrowCompress DefaultArrowDecompress
                         DefaultORCCompress DefaultORCDecompress)
        add_executable(${_t}
            tools/${_t}.cpp
            ${COLUMNAR_BLOB_SOURCES}
            ${ALL_SOURCES}
            ${PMSF_ENTRY_SOURCES}
            ${PMSF_VQMMC_OPERATOR_SOURCES}
            "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp"
            "impl/MMSE/MMSEPostProcessor.cpp")
        target_include_directories(${_t} PUBLIC "src" "impl"
            "impl/PPMC" "impl/VQMMC" "impl/POMAR" "impl/QEM" "impl/RD"
            "impl/MMSE" "impl/APTM" "impl/PCATM" "impl/PCLTTM")
        target_link_libraries(${_t}
            MCGAL_static ZLIB::ZLIB
            Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl
            ${ZSTD_LIBRARY} ${BROTLI_LIBRARIES})
        if(BROTLI_LIBRARIES)
            target_compile_definitions(${_t} PRIVATE PMSF_HAS_BROTLI)
            target_include_directories(${_t} PRIVATE "${BROTLI_INCLUDE_DIR}")
        endif()
        if(DEBUG)
            target_compile_definitions(${_t} PRIVATE DEBUG)
        endif()
    endforeach()

    # （v3-refactor 历史 QEMUnifiedCompress 单表 Bridge demo 已被 pmsf_add_algo_pair(QEM)
    #  自动注册的同名 multi-table target 取代——见下方 pmsf_add_algo_pair 调用。旧的
    #  impl/QEM/QEMUnifiedCompress.cpp 已改写为 `#define PMSF_DEFAULT_BACKEND_UNIFIED +
    #  #include "QEMCompress.cpp"`，与 V2 兄弟风格一致。）

    message(STATUS "Parquet targets enabled (DefaultParquet{,V2}Compress/Decompress + ArrowParquetExample + ${NAME}ColumnarCompress/Decompress + ${NAME}UnifiedCompress/Decompress via pmsf_add_algo_pair)")
endif()

# 框架工具（依赖 PMSF_ENTRY_SOURCES，故必须晚于其定义）。
pmsf_add_tool(RoundExportDecompress tools/RoundExportDecompress.cpp)
pmsf_add_tool(SizeTest tools/SizeTest.cpp)
pmsf_add_tool(AlphabetStreamRoundTripProbe tools/AlphabetStreamRoundTripProbe.cpp)
# Standalone QEM face-budget snapshot (E250 experiments; consumes MCGAL mesh).
pmsf_add_tool(QEMFaceBudget tools/QEMFaceBudget.cpp)
# M5-3: PaperStream v2 multi-stream artifact glue for the QEM progressive seam
# (PSM2 bundle -> P2BL blob container -> `_paper.parquet`).  pmsf_add_tool already
# links ZLIB and (with PARQUET_SUPPORT) Arrow/Parquet/CURL; the storage writer/
# reader translation units are the only missing sources.
target_sources(QEMFaceBudget PRIVATE
    "${CMAKE_SOURCE_DIR}/src/storage/PaperStreamMultiStreamArtifactV2.cpp"
    "${CMAKE_SOURCE_DIR}/src/storage/PaperStreamWriterV2.cpp"
    "${CMAKE_SOURCE_DIR}/src/storage/PaperStreamReaderV2.cpp"
    "${CMAKE_SOURCE_DIR}/src/storage/PaperStreamReader.cpp")
pmsf_add_tool(RDStrictCompress tools/RDStrictCompress.cpp)
pmsf_add_tool(RDStrictDecompress tools/RDStrictDecompress.cpp)
# Full QEM paper algorithm, built from the in-tree headless reference source.
# This is the faithful full-QEM container used for E250 calibration; it is NOT
# the MCGAL approximate QEMFaceBudget path.
add_executable(QEMFaithfulPort reference/OpenMesh_QEM/mesh_simp.cpp)
target_compile_options(QEMFaithfulPort PRIVATE -O2)

# Phase 6 契约测试：Query{roi, max_lod} 与「截断即前缀」。
add_executable(test_truncation_prefix tests/test_truncation_prefix.cpp
    ${ALL_SOURCES}
    ${PMSF_ENTRY_SOURCES}
    ${PMSF_VQMMC_OPERATOR_SOURCES}
    ${COLUMNAR_SOURCES}
    ${UNIFIED_MULTI_TABLE_SOURCES}
    "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp"
    "impl/MMSE/MMSEPostProcessor.cpp")
target_include_directories(test_truncation_prefix PRIVATE src src/common impl
    impl/PPMC impl/VQMMC impl/POMAR impl/QEM impl/RD impl/MMSE impl/APTM impl/PCATM impl/PCLTTM)
target_compile_definitions(test_truncation_prefix PRIVATE COLUMNAR_SUPPORT PMSF_DEFAULT_BACKEND_COLUMNAR)
if(PARQUET_SUPPORT)
    target_link_libraries(test_truncation_prefix PRIVATE
        MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)
else()
    target_link_libraries(test_truncation_prefix PRIVATE MCGAL_static ZLIB::ZLIB)
endif()

add_executable(VisualQualityCompress impl/VisualQuality/VisualQualityCompress.cpp)


# ============ OpenMesh-based R-D progressive compression (self-contained) ============
# See impl/OpenMesh/RD/README.md. This subtree deliberately does not reuse any
# code under src/, deps/MCGAL/, or impl/RD/.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/impl/OpenMesh/RD/CMakeLists.txt")
    add_subdirectory(impl/OpenMesh/RD)
else()
    message(STATUS "OpenMesh RD reference not found; skipping impl/OpenMesh/RD")
endif()

# ============ OpenMesh-based progressive QEM reference (self-contained) ============
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/reference/OpenMesh_PQEM/CMakeLists.txt")
    add_subdirectory(reference/OpenMesh_PQEM)
endif()

# ============ OpenMesh-based reference PPMC (self-contained, CGAL-only) ============
# Maglo et al. 2013. PMSF's PPMCColumnarCompress delegates to this reference
# binary (via ppmc_headless) and packs the resulting .pp3d bitstream into the
# Parquet V2 BLOB variant so the paper reproduces the original PPMC bpv /
# geometric fidelity while still serializing through the relational schema of
# Chapter 4.
add_subdirectory(reference/OpenMesh_PPMC)

# 重命名后 V2/V3 源文件通过 ALL_SOURCES 被直接 add_executable 的测试 target 编译，
# 这些 target 需要链接 Arrow/Parquet/CURL。
if(PARQUET_SUPPORT)
    link_libraries(Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)
endif()

# Test targets below compile ${ALL_SOURCES}, which reference impl/ headers with
# paths relative to the algorithm directories (e.g. APTMConquestCollector.h does
# #include "hooks/APTMConquestGeometryHook.h").  The shipped algorithm targets get
# these include dirs from pmsf_add_algo_pair; mirror them here so every test target
# builds with the same include environment (and hence the same header resolution).
include_directories("src" "impl" "impl/PPMC" "impl/VQMMC" "impl/POMAR" "impl/QEM" "impl/RD"
                    "impl/MMSE" "impl/APTM" "impl/PCATM" "impl/PCLTTM")

# ============ Phase 10 hook-surface smoke tests ============
# Verifies that ErrorMetric / Predictor / EntropyCoder / Codebook headers
# round-trip correctly. Not linked into any algorithm; pure additive gate.
add_executable(test_hooks tests/test_hooks.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_hooks PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Phase 11 Parquet V3 smoke test ============
# End-to-end Parquet V3 round-trip with dynamic side channels. Only built
# when Arrow/Parquet are available.
if(PARQUET_SUPPORT)
    add_executable(test_unified tests/test_unified.cpp ${UNIFIED_SOURCES} ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
    target_link_libraries(test_unified PRIVATE
        MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)

    add_executable(verify_qem_unified_roundtrip tests/verify_qem_unified_roundtrip.cpp ${UNIFIED_SOURCES} ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
    target_link_libraries(verify_qem_unified_roundtrip PRIVATE
        MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)

    # ============================================================
    # PaperStream v1 storage contract (M1): writer/reader + synthetic
    # round-trip / prefix / corruption tests.  No MCGAL, no algorithm.
    # ============================================================
    add_executable(test_paper_stream_storage
        tests/test_paper_stream_storage.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage PRIVATE CURL::libcurl)
    endif()

    # M4-1 additive PaperStream v2 storage contract: v2 table fields + new
    # codec/dictionary headers only. Deliberately links NO Arrow/Parquet and
    # does not use the v1 bridge writer/reader.
    add_executable(test_paper_stream_storage_v2
        tests/test_paper_stream_storage_v2.cpp)
    target_include_directories(test_paper_stream_storage_v2 PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2 PRIVATE ZLIB::ZLIB)

    # M4-2 opt-in PaperStream v2 writer/reader: compact v2 schema, RG-level
    # random access, v1/v2 dispatch. Compiles the v2 I/O sources locally so the
    # main build (COLUMNAR_SOURCES) is untouched until M4-3/M4-4 integrate them.
    add_executable(test_paper_stream_storage_v2_io
        tests/test_paper_stream_storage_v2_io.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage_v2_io PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2_io PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage_v2_io PRIVATE CURL::libcurl)
    endif()

    # M4-4 native-payload storage contract: PNPL/PPTH-v2 per-round/per-LOD
    # payloads, prefix/RA, whole-frame fallback, deterministic bytes, fail-closed.
    add_executable(test_paper_stream_storage_v2_native
        tests/test_paper_stream_storage_v2_native.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage_v2_native PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2_native PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage_v2_native PRIVATE CURL::libcurl)
    endif()

    # M5-1 zero-overhead v2 container (single `paper_blob` column): PPMC row.
    add_executable(test_paper_stream_storage_v2_blob
        tests/test_paper_stream_storage_v2_blob.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage_v2_blob PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2_blob PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage_v2_blob PRIVATE CURL::libcurl)
    endif()

    # M5-3 PSM2 multi-stream bundle -> P2BL blob container (QEM progressive).
    add_executable(test_paper_stream_storage_v2_multistream
        tests/test_paper_stream_storage_v2_multistream.cpp
        src/storage/PaperStreamMultiStreamArtifactV2.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage_v2_multistream PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2_multistream PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage_v2_multistream PRIVATE CURL::libcurl)
    endif()

    # M6-RA (task-37): blob-container random-access evidence (readPrefix/LOD/
    # row-group, bytesRead, fail-closed, determinism).
    add_executable(test_phase2_ab_ra
        tests/test_phase2_ab_ra.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_phase2_ab_ra PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_phase2_ab_ra PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_phase2_ab_ra PRIVATE CURL::libcurl)
    endif()

    # M5-5 opaque-payload PaperStream v2 artifact (VQMMC: verbatim binary
    # container 777 B header/base + 63,733 B CONN, optional payload codec).
    add_executable(test_paper_stream_storage_v2_opaque
        tests/test_paper_stream_storage_v2_opaque.cpp
        src/storage/PaperStreamOpaquePayloadV2.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_storage_v2_opaque PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_storage_v2_opaque PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_storage_v2_opaque PRIVATE CURL::libcurl)
    endif()

    # M4-3 opt-in v2 bridge: PMSF_PAPERSTREAM_V2 switch + PaperBridgeV2 helpers
    # (the operator objects themselves need the collector/Columnar stack and are
    # validated by the live A/B gates after apply).
    add_executable(test_paper_stream_serialize_v2_bridge
        tests/test_paper_stream_serialize_v2_bridge.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_serialize_v2_bridge PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_serialize_v2_bridge PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_serialize_v2_bridge PRIVATE CURL::libcurl)
    endif()

    add_executable(test_paper_stream_random_access
        tests/test_paper_stream_random_access.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_paper_stream_random_access PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(test_paper_stream_random_access PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_random_access PRIVATE CURL::libcurl)
    endif()

    add_executable(bench_paper_stream_footer
        tests/bench_paper_stream_footer.cpp
        src/storage/PaperStreamWriter.cpp)
    target_include_directories(bench_paper_stream_footer PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(bench_paper_stream_footer PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(bench_paper_stream_footer PRIVATE CURL::libcurl)
    endif()

    add_executable(PaperPrefixBpv
        tools/PaperPrefixBpvParquet.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamOpaquePayloadV2.cpp)
    target_include_directories(PaperPrefixBpv PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json")
    target_link_libraries(PaperPrefixBpv PRIVATE
        ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(PaperPrefixBpv PRIVATE CURL::libcurl)
    endif()

    add_executable(test_paper_stream_symbol_adapter
        tests/test_paper_stream_symbol_adapter.cpp
        src/storage/PaperStreamWriter.cpp
        src/storage/PaperStreamReader.cpp
        src/rangecoder/rangecod.c
        src/rangecoder/qsmodel.c)
    target_include_directories(test_paper_stream_symbol_adapter PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/deps/json"
        "${CMAKE_SOURCE_DIR}/deps/MCGAL/include")
    target_link_libraries(test_paper_stream_symbol_adapter PRIVATE
        MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(TARGET CURL::libcurl)
        target_link_libraries(test_paper_stream_symbol_adapter PRIVATE CURL::libcurl)
    endif()
endif()

# ============ Phase A/B: 示例 QEM metric hook 共享库 ============
# 不链接 host 符号，仅依赖 C-ABI 头与 MCGAL 内联访问器；通过 dlopen 由 PMSFCompress 加载。
add_library(qem_metric_hook SHARED hooks/qem_metric/QEMMetricHook.cpp)
target_include_directories(qem_metric_hook PUBLIC src deps/json deps/MCGAL/include /usr/local/include/OpenMesh/include)
set_target_properties(qem_metric_hook PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============ Phase 12 capability / property registrar smoke test ============
add_executable(test_capability tests/test_capability.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_capability PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Phase 13 (Phase A.7) curated hook variant smoke test ============
# Compile-time witness that every new curated hook header (Butterfly /
# DualParallelogram predictors, ContextAdaptiveRangeCoder, SR-VQ codebook,
# CurvatureHysteresis / Cluster / Seam feature detectors, QuadricWithNormalWeight
# metric) parses under the framework's C++20 settings and round-trips encode/
# decode for the entropy coders + codebooks. No algorithm depends on this
# target — purely a refactor gate.
add_executable(test_hook_variants tests/test_hook_variants.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_hook_variants PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Phase A.2: BFSTraversal driver witness ============
# Ensures the shared BFS driver declared in src/core/BFSTraversal.h is not
# just declared but actually exercised on the Stanford bunny. Validates
# determinism (two runs → same counts) and all four helper APIs.
add_executable(test_bfs_traversal tests/test_bfs_traversal.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_bfs_traversal PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Phase 0 (connectivity-bitstream) smoke tests ============
# 纯 header/cpp round-trip，不依赖 MCGAL 以外的任何算法路径。统一和
# test_capability / test_hooks 一致，链入 ${ALL_SOURCES} 以复用 BitStream 源文件。
# 详见 docs/CONNECTIVITY_BITSTREAM_IMPLEMENTATION_PLAN.md §0.2, §0.3。
add_executable(test_bit_stream tests/test_bit_stream.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_bit_stream PRIVATE MCGAL_static ZLIB::ZLIB)

add_executable(test_connectivity_blob tests/test_connectivity_blob.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_connectivity_blob PRIVATE MCGAL_static ZLIB::ZLIB)

# Phase 0 · Step 0.4 —— EliminateOperator round-list hook smoke.
add_executable(test_patch_list_exposure tests/test_patch_list_exposure.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_patch_list_exposure PRIVATE MCGAL_static ZLIB::ZLIB)

# Phase 1 · Step 1.1 —— ValencePatchOnlySymbolCollectOperator alphabet/container smoke.
# 真实 BFS 路径的 round-trip 测试要等 Step 1.2 + 1.4 接入 PCLTTMValence Columnar 后再做。
add_executable(test_valence_patch_only tests/test_valence_patch_only.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_valence_patch_only PRIVATE MCGAL_static ZLIB::ZLIB)

# Canonical facet/edge/vertex/point + side-channel round-trip smoke.
add_executable(test_canonical_symbol_roundtrip tests/test_canonical_symbol_roundtrip.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_link_libraries(test_canonical_symbol_roundtrip PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Paper-alignment PR-1 (PCLTTM Tag, BlobLayout=16, path B) ============
# - test_vertex_tag       : MCGAL::Vertex::tagFlag_ smoke (4 inline 方法 + reset 不复位)
# - test_pclttm_tag_table : 加载 docs/paper_alignment/pclttm_tag_table_groundtruth.csv
#                           对每行断言 pmsf::pclttm::evolve(...) 与 ground truth 一致。
# 详见 docs/paper_alignment/PCLTTM_OFFICIAL_REPRO_PROMPT.md PR-1 §R3-R5。
add_executable(test_vertex_tag tests/test_vertex_tag.cpp)
target_link_libraries(test_vertex_tag PRIVATE MCGAL_static)
target_include_directories(test_vertex_tag PRIVATE deps/MCGAL/include)

add_executable(test_pclttm_tag_table tests/test_pclttm_tag_table.cpp)
target_include_directories(test_pclttm_tag_table PRIVATE src src/common)

# ============ Paper-alignment PR-2 (PCLTTM Tag, BlobLayout=16, path B) ============
# - test_pclttm_tag_evolution : encoder/decoder shared applyTagEvolution() helper
#   对称性 witness。覆盖 N=3..6 × tagLeft × tagRight 36 case + 30-round 重复序列。
add_executable(test_pclttm_tag_evolution tests/test_pclttm_tag_evolution.cpp)
target_include_directories(test_pclttm_tag_evolution PRIVATE src src/common)

# ============ Paper-alignment PR-1 (PCATM ZShape, BlobLayout=15, path B) ============
# - test_pcatm_zshape_geometry : N=4..9 ground truth + Figure 2 (4) mirror Z
#                                illegal 反例。详见
#                                docs/paper_alignment/pcatm_zshape_groundtruth.md。
add_executable(test_pcatm_zshape_geometry tests/test_pcatm_zshape_geometry.cpp)
target_include_directories(test_pcatm_zshape_geometry PRIVATE src src/common)

# ============ PCATM paper mode-2 wire-visible matcher ============
# Shared encoder/decoder geometry recovery witness.  This test only depends on
# MCGAL topology and the header-only matcher, so it stays independent of the
# Parquet backend and is cheap enough for local protocol regression.
add_executable(test_pcatm_paper_mode2 tests/test_pcatm_paper_mode2.cpp)
target_include_directories(test_pcatm_paper_mode2 PRIVATE src src/common impl)
target_link_libraries(test_pcatm_paper_mode2 PRIVATE MCGAL_static)

# ============ Paper-alignment PR-2 (PCATM ZShape, BlobLayout=15, path B) ============
# - test_alternating_invariant : AlternatingInvariantChecker 校验 + 与
#   computeZShape 联动；PCATM 论文 §4 核心不变量。
add_executable(test_alternating_invariant tests/test_alternating_invariant.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_include_directories(test_alternating_invariant PRIVATE src src/common)
target_link_libraries(test_alternating_invariant PRIVATE MCGAL_static ZLIB::ZLIB)

# ============ Phase 5 PR-J2.1 simulate-decoder spike ============
# 验证 "decoder 仅凭 valence 流 + 拓扑 + halfedge.isAdded 即可复现 ValenceCode
# 序列" 的命题。pure additive：不修改任何 production / MCGAL 代码，只新增
# tests/test_simulate_decoder_spike.cpp 一个文件。
# 详见 docs/paper_alignment/PHASE5_PR_J2_DESIGN.md §3 PR-J2.1。
add_executable(test_simulate_decoder_spike tests/test_simulate_decoder_spike.cpp ${ALL_SOURCES} ${PMSF_ENTRY_SOURCES} ${PMSF_VQMMC_OPERATOR_SOURCES} "src/config/PMSFConfigParser.cpp" "src/hook/HookRegistry.cpp" "impl/MMSE/MMSEPostProcessor.cpp")
target_include_directories(test_simulate_decoder_spike PRIVATE src src/common impl/PCLTTM)
target_link_libraries(test_simulate_decoder_spike PRIVATE MCGAL_static ZLIB::ZLIB)

# Paper-equation QEM regression: value mesh + framework operation sequence,
# independent of the halfedge/Arrow implementations used by legacy QEM.
add_executable(test_qem_paper_engine tests/test_qem_paper_engine.cpp)
target_include_directories(test_qem_paper_engine PRIVATE src impl)
set_target_properties(test_qem_paper_engine PROPERTIES LINK_LIBRARIES "")

# Signed MSB-first residual grammar shared by conquest protocols.
add_executable(test_signed_bitplane_coder tests/test_signed_bitplane_coder.cpp
    src/rangecoder/rangecod.c src/rangecoder/qsmodel.c)
target_include_directories(test_signed_bitplane_coder PRIVATE src)
set_target_properties(test_signed_bitplane_coder PROPERTIES LINK_LIBRARIES "z")

# Deterministic connectivity-only recursive segmentation, independent of MCGAL.
add_executable(test_recursive_topology_bisection tests/test_recursive_topology_bisection.cpp)
target_include_directories(test_recursive_topology_bisection PRIVATE src)
set_target_properties(test_recursive_topology_bisection PROPERTIES LINK_LIBRARIES "")

# Rooted combinatorial numbering independent of allocation and geometry.
add_executable(test_conquest_layer_topology tests/test_conquest_layer_topology.cpp src/core/MeshRuntime.cpp src/conquest/ConquestDecodeLayer.cpp)
target_include_directories(test_conquest_layer_topology PRIVATE src impl)
target_link_libraries(test_conquest_layer_topology PRIVATE MCGAL_static)

# Logical MMSE symbols survive role-based storage independently of physical I/O.
add_executable(test_conquest_patch_discovery tests/test_conquest_patch_discovery.cpp src/core/MeshRuntime.cpp src/conquest/FrontierConquestEngine.cpp src/conquest/ConquestDecodeLayer.cpp impl/common/PMSFRetriangulation.cpp)
target_include_directories(test_conquest_patch_discovery PRIVATE src impl)
target_link_libraries(test_conquest_patch_discovery PRIVATE MCGAL_static ZLIB::ZLIB)

add_executable(test_edgebreaker_domain tests/test_edgebreaker_domain.cpp)
target_include_directories(test_edgebreaker_domain PRIVATE src)
set_target_properties(test_edgebreaker_domain PROPERTIES LINK_LIBRARIES "")

add_executable(test_pclttm_conquest_symbols tests/test_pclttm_conquest_symbols.cpp src/rangecoder/rangecod.c src/rangecoder/qsmodel.c)
target_include_directories(test_pclttm_conquest_symbols PRIVATE src impl deps/MCGAL/include)
set_target_properties(test_pclttm_conquest_symbols PROPERTIES LINK_LIBRARIES "z")

add_executable(test_pclttm_fig9 tests/test_pclttm_fig9.cpp src/core/MeshRuntime.cpp src/conquest/FrontierConquestEngine.cpp impl/common/PMSFRetriangulation.cpp)
target_include_directories(test_pclttm_fig9 PRIVATE src impl)
target_link_libraries(test_pclttm_fig9 PRIVATE MCGAL_static ZLIB::ZLIB)

add_executable(test_pclttm_frenet tests/test_pclttm_frenet.cpp src/rangecoder/rangecod.c src/rangecoder/qsmodel.c)
target_include_directories(test_pclttm_frenet PRIVATE src impl)
set_target_properties(test_pclttm_frenet PROPERTIES LINK_LIBRARIES "z")

add_executable(test_mmse_fine_ring tests/test_mmse_fine_ring.cpp)
target_include_directories(test_mmse_fine_ring PRIVATE src)
set_target_properties(test_mmse_fine_ring PROPERTIES LINK_LIBRARIES "")

add_executable(test_mmse_layer_symbols tests/test_mmse_layer_symbols.cpp
    src/rangecoder/rangecod.c src/rangecoder/qsmodel.c)
target_include_directories(test_mmse_layer_symbols PRIVATE src impl deps/MCGAL/include)
set_target_properties(test_mmse_layer_symbols PROPERTIES LINK_LIBRARIES "z")

# ============ M1-b Step 2 (PCATM Paper, BlobLayout=20) encoder smoke ============
# 验证 PCATMPaperEliminateOperator 真 Z-shape 三角化在 bunny 5 round 上不崩，
# 配套 PCATMZShapeGeometry::computeZShape (PR-1 ground truth helper) + alternating
# invariant 校验（论文 §4 invariant）。pure additive：仅新增 4 个文件，
# 9 算法 baseline (含 PCATM/PCATMColor/PCATMZColor) md5 字节级零回归。
# 详见 docs/paper_alignment/pcatm_zshape_groundtruth.md。
if(PARQUET_SUPPORT)
    add_executable(test_pcatm_paper_encoder_zshape tests/test_pcatm_paper_encoder_zshape.cpp
        ${COLUMNAR_SOURCES}
        ${UNIFIED_MULTI_TABLE_SOURCES}
        ${ALL_SOURCES}
        ${PMSF_ENTRY_SOURCES}
        ${PMSF_VQMMC_OPERATOR_SOURCES}
        "src/config/PMSFConfigParser.cpp"
        "src/hook/HookRegistry.cpp"
        "impl/MMSE/MMSEPostProcessor.cpp")
    target_include_directories(test_pcatm_paper_encoder_zshape PRIVATE src src/common impl
        impl/PPMC impl/VQMMC impl/POMAR impl/QEM impl/RD impl/MMSE impl/APTM impl/PCATM impl/PCLTTM)
    target_compile_definitions(test_pcatm_paper_encoder_zshape PRIVATE COLUMNAR_SUPPORT PMSF_DEFAULT_BACKEND_COLUMNAR)
    target_link_libraries(test_pcatm_paper_encoder_zshape PRIVATE
        MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared CURL::libcurl)
endif()

get_property(targetIncludes DIRECTORY PROPERTY INCLUDE_DIRECTORIES)
message(STATUS "Target Include Directories: ${targetIncludes}")

# M6 page-level RA spike (task-53): opt-in PRA1 chunked profile test tool.
if(PARQUET_SUPPORT)
    add_executable(test_m6_page_ra
        tests/test_m6_page_ra.cpp
        src/storage/PaperStreamWriterV2.cpp
        src/storage/PaperStreamReaderV2.cpp
        src/storage/PaperStreamReader.cpp)
    target_include_directories(test_m6_page_ra PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/src/include"
        "${CMAKE_SOURCE_DIR}/impl"
        "${CMAKE_SOURCE_DIR}/impl/QEM"
        "${CMAKE_SOURCE_DIR}/deps/json"
        "${CMAKE_SOURCE_DIR}/deps/MCGAL/include"
        "${CMAKE_SOURCE_DIR}/deps/MCGAL/include/openmesh"
        "${CMAKE_SOURCE_DIR}/deps/MCGAL/include/openmesh/System")
    target_compile_definitions(test_m6_page_ra PRIVATE COLUMNAR_SUPPORT PMSF_HAS_ZSTD PMSF_HAS_BROTLI)
    target_link_libraries(test_m6_page_ra PRIVATE MCGAL_static ZLIB::ZLIB Arrow::arrow_shared Parquet::parquet_shared)
    if(ZSTD_LIBRARY)
        target_link_libraries(test_m6_page_ra PRIVATE "${ZSTD_LIBRARY}")
    endif()
endif()
