cmake_minimum_required(VERSION 3.10)
project(vehicle_localization C)

# 设置C标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ========== 构建类型配置 ==========
# 公共编译选项 - 根据编译器设置警告级别
if(MSVC)
    # MSVC: /W4 = extra warnings (equivalent to -Wextra)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
else()
    # GCC/Clang: -Wall -Wextra
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
endif()

# 设置可用的构建类型和默认值
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build" FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;ReleaseWithDebugInfo")

# Debug 构建：最大调试支持，无优化
# GCC:  -g: 包含调试符号，-O0: 无优化，-DDEBUG: 定义DEBUG宏
# MSVC: /Zi: 包含调试符号，/Od: 无优化
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    if(MSVC)
        set(CMAKE_C_FLAGS_DEBUG "/Zi /Od -DDEBUG")
    else()
        set(CMAKE_C_FLAGS_DEBUG "-g -O0 -DDEBUG")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG}")
endif()

# Release 构建：最大优化，无调试信息
# GCC:  -O3: 最高级别优化，-DNDEBUG: 禁用断言
# MSVC: /O2: 优化，/DNDEBUG: 禁用断言
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        set(CMAKE_C_FLAGS_RELEASE "/O2 -DNDEBUG")
    else()
        set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
endif()

# ReleaseWithDebugInfo 构建：中等优化 + 调试符号
# GCC:  -g: 调试符号，-O2: 中级优化，-DNDEBUG: 禁用断言
# MSVC: /Zi: 调试符号，/O2: 优化，-DNDEBUG: 禁用断言
if(CMAKE_BUILD_TYPE STREQUAL "ReleaseWithDebugInfo")
    if(MSVC)
        set(CMAKE_C_FLAGS_RELWITHDEBINFO "/Zi /O2 -DNDEBUG")
    else()
        set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -O2 -DNDEBUG")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
endif()

# MSVC 不需要单独链接 m.lib，数学库已内置在 C 运行时
if(MSVC)
    set(MATH_LIB "")
else()
    set(MATH_LIB m)
endif()

# 项目版本
set(PROJECT_VERSION 1.0.0)
set(PROJECT_DESCRIPTION "EKF-based Multi-Sensor Vehicle Localization System")

# 设置输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 源文件列表
set(SOURCES
    src/motion_model.c
    src/kalman_filter.c
    src/steering_model.c
    src/wheel_odometry_processor.c
    src/steering_gain_calibration.c
    tools/rtk_calibration.c
    src/localization_system.c
)

# 头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/tools)

# 创建库目标
add_library(localization_lib ${SOURCES})

# 链接数学库
target_link_libraries(localization_lib PRIVATE ${MATH_LIB})

# 可执行文件 - 定位系统示例
add_executable(localization_example example_usage.c)
target_link_libraries(localization_example PRIVATE localization_lib ${MATH_LIB})

# 可执行文件 - 标定CSV数据生成工具
add_executable(generate_calibration_csv tools/generate_calibration_csv.c)
target_link_libraries(generate_calibration_csv PRIVATE ${MATH_LIB})

# 可执行文件 - 转向映射表生成工具
add_executable(generate_steering_mapping tools/generate_steering_mapping.c)
target_link_libraries(generate_steering_mapping PRIVATE ${MATH_LIB})

# 可执行文件 - 离线标定工具（超简化版，无依赖）
# add_executable(offline_calibration tools/offline_calibration.c)


# 可执行文件 - 转向角映射表集成示例
add_executable(example_mapping_integration example_mapping_integration.c src/steering_model.c src/motion_model.c)
target_link_libraries(example_mapping_integration PRIVATE ${MATH_LIB})

# 测试可执行文件 - Task 4: 转向模型与RTK标定集成测试
# add_executable(test_steering_rtk_integration tools/test_steering_rtk_integration.c)
# target_link_libraries(test_steering_rtk_integration PRIVATE localization_lib m)

# 测试可执行文件 - Phase 2 RTK标定集成测试
# add_executable(test_rtk_phase2_integration tools/test_rtk_phase2_integration.c)
# target_link_libraries(test_rtk_phase2_integration PRIVATE localization_lib m)

# 离线标定工具 - 从 CSV 文件进行标定
# add_executable(offline_calibration_from_csv
#     tools/offline_calibration_csv.c
#     tools/main_offline_calibration.c
# )
# target_link_libraries(offline_calibration_from_csv PRIVATE localization_lib m)

# 安装目标
install(TARGETS localization_example
    RUNTIME DESTINATION bin
)

install(TARGETS localization_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    src/localization_types.h
    src/motion_model.h
    src/kalman_filter.h
    tools/rtk_calibration.h
    src/localization_system.h
    src/localization_system_private.h
        tools/wheel_odometry_processor.h.bak
    DESTINATION include
)

# 打印构建信息
message(STATUS "========================================")
message(STATUS "Vehicle Localization System")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "C Standard: C99")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "========================================")

# 显示编译标志和构建类型详情
message(STATUS "")
message(STATUS "📋 Build Configuration Details:")
message(STATUS "  Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "  C Standard: C99")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")

# 根据构建类型显示对应的编译标志说明
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "  Configuration: Debug Build")
    message(STATUS "    • Optimization: -O0 (No optimization)")
    message(STATUS "    • Debug Info: -g (Full debug symbols)")
    message(STATUS "    • Macro: -DDEBUG (Debug macros enabled)")
    message(STATUS "    • Use Case: Development & Debugging")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    message(STATUS "  Configuration: Release Build")
    message(STATUS "    • Optimization: -O3 (Maximum optimization)")
    message(STATUS "    • Debug Info: None (Stripped)")
    message(STATUS "    • Macro: -DNDEBUG (Assertions disabled)")
    message(STATUS "    • Use Case: Production & Performance")
elseif(CMAKE_BUILD_TYPE STREQUAL "ReleaseWithDebugInfo")
    message(STATUS "  Configuration: ReleaseWithDebugInfo Build")
    message(STATUS "    • Optimization: -O2 (Balanced optimization)")
    message(STATUS "    • Debug Info: -g (Debug symbols for profiling)")
    message(STATUS "    • Macro: -DNDEBUG (Assertions disabled)")
    message(STATUS "    • Use Case: Production with debugging capability")
endif()

message(STATUS "")
message(STATUS "📁 Output Directories:")
message(STATUS "  Executables: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "  Libraries: ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "")
message(STATUS "🚀 Usage:")
message(STATUS "  Debug:               cmake -DCMAKE_BUILD_TYPE=Debug ..")
message(STATUS "  Release:             cmake -DCMAKE_BUILD_TYPE=Release ..")
message(STATUS "  ReleaseWithDebugInfo: cmake -DCMAKE_BUILD_TYPE=ReleaseWithDebugInfo ..")
message(STATUS "========================================")
