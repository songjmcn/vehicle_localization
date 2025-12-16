cmake_minimum_required(VERSION 3.10)
project(vehicle_localization C)

# 设置C标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# 可选：Debug构建添加调试信息
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -DDEBUG")
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
    motion_model.c
    kalman_filter.c
    rtk_calibration.c
    localization_system.c
    wheel_odometry_processor.c
)

# 头文件目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 创建库目标
add_library(localization_lib ${SOURCES})

# 链接数学库
target_link_libraries(localization_lib PRIVATE m)

# 可执行文件
add_executable(localization_example example_usage.c)
target_link_libraries(localization_example PRIVATE localization_lib m)

# 安装目标
install(TARGETS localization_example
    RUNTIME DESTINATION bin
)

install(TARGETS localization_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES
    localization_types.h
    motion_model.h
    kalman_filter.h
    rtk_calibration.h
    localization_system.h
    localization_system_private.h
    wheel_odometry_processor.h
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

# 显示编译标志
message(STATUS "Compiler Flags: ${CMAKE_C_FLAGS}")
message(STATUS "Output Directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
