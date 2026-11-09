cmake_minimum_required(VERSION 3.20)
project(osmo_360_stitcher LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PkgConfig REQUIRED)

pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavformat
    libavcodec
    libavutil
    libswscale
)

pkg_check_modules(MLT REQUIRED IMPORTED_TARGET
    mlt-framework-7
)

# Vulkan setup: use third_party headers + system libvulkan.so
set(VULKAN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Vulkan-Headers/include)
find_library(VULKAN_LIB vulkan REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${VULKAN_INCLUDE_DIR}
    ${MLT_INCLUDE_DIRS}
)

# Custom command to compile GLSL compute shader to SPIR-V
set(SHADER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/shaders/osv_stitch.comp)
set(SHADER_SPV ${CMAKE_CURRENT_SOURCE_DIR}/shaders/osv_stitch.spv)

add_custom_command(
    OUTPUT ${SHADER_SPV}
    COMMAND glslc ${SHADER_SRC} -o ${SHADER_SPV}
    DEPENDS ${SHADER_SRC}
    COMMENT "Compiling Vulkan compute shader osv_stitch.comp -> osv_stitch.spv"
)

add_custom_target(compile_shaders ALL DEPENDS ${SHADER_SPV})

# Core Library
add_library(osv_core STATIC
    src/osv_metadata.cpp
    src/osv_vulkan.cpp
)
add_dependencies(osv_core compile_shaders)

target_link_libraries(osv_core PUBLIC
    ${VULKAN_LIB}
    PkgConfig::FFMPEG
)

# CLI Tool
add_executable(osv_stitch
    src/main.cpp
)

target_link_libraries(osv_stitch PRIVATE
    osv_core
    ${VULKAN_LIB}
    PkgConfig::FFMPEG
)

# MLT Service Plugin (libmltosv.so)
add_library(mltosv MODULE
    mlt-plugin/mlt_osv.cpp
    mlt-plugin/producer_osv.cpp
)

target_link_libraries(mltosv PRIVATE
    osv_core
    ${VULKAN_LIB}
    PkgConfig::FFMPEG
    PkgConfig::MLT
)

set_target_properties(mltosv PROPERTIES
    PREFIX "lib"
    OUTPUT_NAME "mltosv"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugins"
)

# Setup MLT runtime environment (symlinks to system plugins + loader.dict)
add_custom_target(setup_mlt_env ALL
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/setup_mlt_env.sh "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Setting up MLT environment symlinks and loader.dict"
)
add_dependencies(mltosv setup_mlt_env)

# High-Performance Frei0r Bigsh0t Equirectangular to Rectilinear Filter
find_package(OpenMP REQUIRED)

add_library(bigsh0t_eq_to_rect MODULE
    src/bigsh0t_eq_to_rect.cpp
)

target_link_libraries(bigsh0t_eq_to_rect PRIVATE
    OpenMP::OpenMP_CXX
)

target_compile_options(bigsh0t_eq_to_rect PRIVATE
    -O3
    -ffast-math
    -mavx2
    -mfma
)

set_target_properties(bigsh0t_eq_to_rect PROPERTIES
    PREFIX ""
    OUTPUT_NAME "bigsh0t_eq_to_rect"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/plugins/frei0r"
)

