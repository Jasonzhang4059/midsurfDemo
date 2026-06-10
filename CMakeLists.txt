cmake_minimum_required(VERSION 3.20)
project(MidSurfDemo VERSION 1.0 LANGUAGES CXX)
set(CMAKE_BUILD_PARALLEL_LEVEL 8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ======== Qt 自动处理工具（开启 moc/uic/rcc）========
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ======== Qt 配置（使用绝对路径 + find_package）========
set(QT_PREFIX_PATH "C:\Qt\Qt5.9.1\5.9.1\msvc2017_64")
set(CMAKE_PREFIX_PATH ${QT_PREFIX_PATH})

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(QT_LIBS
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

# ======== 第三方库根目录 ==========
set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/3rdParty)

# ======== OpenCASCADE 配置 ==========
set(OCC_ROOT ${THIRD_PARTY_DIR}/occ)

set(OCC_LIBS
    TKernel TKMath TKService TKV3d TKOpenGL TKBrep TKIGES TKSTL TKVRML TKSTEP
    TKSTEPAttr TKSTEP209 TKSTEPBase TKGeomBase TKGeomAlgo TKG3d TKG2d TKXSBase
    TKShHealing TKHLR TKTopAlgo TKMesh TKPrim TKCDF TKBool TKBO TKFillet
    TKOffset TKXDESTEP
)

# ======== OpenVDB 配置 ==========
set(OPENVDB_ROOT ${THIRD_PARTY_DIR}/openvdb)

# ======== Eigen 配置（头文件库）=========
set(EIGEN_ROOT ${THIRD_PARTY_DIR}/eigen-3.3.8)

# ======== NLopt 配置 ==========
set(NLOPT_ROOT ${THIRD_PARTY_DIR}/nlopt)

# ======== 添加 imso 源文件 ==========
set(IMSO_SRC_DIR ${CMAKE_SOURCE_DIR}/src/imso)

set(IMSO_SOURCE_FILES
    ${IMSO_SRC_DIR}/occIgesTrans.cpp
    ${IMSO_SRC_DIR}/continuityModification.cpp
    ${IMSO_SRC_DIR}/continuitymodification.h
    ${IMSO_SRC_DIR}/FittingBSpline.cpp
    ${IMSO_SRC_DIR}/globalFunc.cpp
    ${IMSO_SRC_DIR}/MathUtil.cpp
    ${IMSO_SRC_DIR}/PolyIGA.cpp
    ${IMSO_SRC_DIR}/RWGeometric.cpp
    ${IMSO_SRC_DIR}/Spline.cpp
    ${IMSO_SRC_DIR}/SplineSurface.cpp
    ${IMSO_SRC_DIR}/SplineVolume.cpp
    ${IMSO_SRC_DIR}/XBaseMesh.cpp
    ${IMSO_SRC_DIR}/XData.cpp
    ${IMSO_SRC_DIR}/XFunction.cpp
    ${IMSO_SRC_DIR}/XIOStream.cpp
)

# ======== 原有 src 目录文件查找 ==========
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB SOURCE_FILE ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB RESOURCE_FILE ${PROJECT_SOURCE_DIR}/src/*.qrc)

source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${HEADER_FILES})
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${SOURCE_FILE})
source_group(TREE ${CMAKE_SOURCE_DIR} FILES ${RESOURCE_FILE})

# ======== 创建可执行目标 ==========
add_executable(${PROJECT_NAME} 
    ${HEADER_FILES} 
    ${SOURCE_FILE} 
    ${RESOURCE_FILE}
    ${IMSO_SOURCE_FILES}
)

# ======== 添加所有头文件搜索路径 ==========
target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_SOURCE_DIR}/src
    PRIVATE ${IMSO_SRC_DIR}
    PRIVATE ${OCC_ROOT}/inc
    PRIVATE ${OPENVDB_ROOT}           # openvdb.h 在 openvdb/openvdb.h，需要父目录
    PRIVATE ${OPENVDB_ROOT}/include   # 如果有额外 include
    PRIVATE ${EIGEN_ROOT}
    PRIVATE ${NLOPT_ROOT}
)

# ======== 添加库路径 ==========
target_link_directories(${PROJECT_NAME}
    PRIVATE ${OCC_ROOT}/lib
    PRIVATE ${OPENVDB_ROOT}/lib
    PRIVATE ${NLOPT_ROOT}
)

# ======== 链接所有库 ==========
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${QT_LIBS}
    ${OCC_LIBS}
    openvdb
    libnlopt-0
)

# ======== Visual Studio 启动项目设置 ========
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# ======== MSVC 专用编译优化设置 ========
if (MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /bigobj)
endif()