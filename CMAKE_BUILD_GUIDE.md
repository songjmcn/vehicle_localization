# CMake 构建系统使用指南

## 概述

本项目使用 **CMake 3.10+** 作为跨平台构建系统，支持 Linux、macOS 和 Windows。

**优点**：
- ✓ 跨平台构建支持
- ✓ 自动依赖管理
- ✓ IDE集成友好（VS Code、CLion、Visual Studio等）
- ✓ 灵活的编译配置
- ✓ 库和可执行文件分离管理

---

## 快速开始

### 方法1：使用自动化脚本（推荐）

```bash
# 进入项目目录
cd /Users/alex/workspace/code_review/vehicle_localization

# Release构建（优化编译，推荐）
./cmake_build.sh build

# 或Debug构建（包含调试信息）
./cmake_build.sh build-debug

# 运行程序
./cmake_build.sh run

# 清除编译产物
./cmake_build.sh clean
```

### 方法2：手动CMake命令

```bash
# 创建构建目录
mkdir build && cd build

# 生成Makefile (Release模式)
cmake -DCMAKE_BUILD_TYPE=Release ..

# 或 Debug模式
cmake -DCMAKE_BUILD_TYPE=Debug ..

# 编译
make -j$(nproc)

# 运行
./bin/localization_example

# 清除
cd .. && rm -rf build
```

### 方法3：使用IDE

#### VS Code
1. 安装 CMake Tools 扩展
2. 打开项目文件夹
3. 点击底部状态栏的 CMake 图标
4. 选择工具链和构建类型
5. 按 F7 构建

#### CLion
1. 打开项目
2. CLion自动识别CMakeLists.txt
3. 选择构建配置（Debug/Release）
4. 按 Ctrl+F9 构建

---

## 脚本命令详解

### `./cmake_build.sh build`
**功能**：以Release模式构建项目
- 优化代码以获得最佳性能
- 移除调试符号以减小文件大小
- 推荐用于生产环境

**输出**：
```
build/
├── bin/
│   └── localization_example     # 可执行文件
├── lib/
│   └── liblocalization_lib.a    # 静态库
└── CMakeFiles/                  # CMake缓存
```

### `./cmake_build.sh build-debug`
**功能**：以Debug模式构建项目
- 包含完整的调试符号
- 不优化代码以便调试
- 推荐用于开发调试

**输出**：同build，但可执行文件包含调试信息

### `./cmake_build.sh clean`
**功能**：删除所有编译产物
- 删除整个 `build/` 目录
- 恢复到初始状态

### `./cmake_build.sh rebuild`
**功能**：清除后重新构建（clean + build）
- 强制重新编译所有文件
- 用于解决奇怪的构建问题

### `./cmake_build.sh install`
**功能**：安装编译结果
- 复制可执行文件到系统路径
- 需要有安装权限

**注意**：在macOS上可能需要使用 `sudo`

### `./cmake_build.sh run`
**功能**：运行可执行程序
- 自动运行 `build/bin/localization_example`
- 如果未编译会报错

### `./cmake_build.sh test`
**功能**：运行测试
- 当前等同于 `run` 命令
- 可扩展为运行多个测试

### `./cmake_build.sh help`
**功能**：显示帮助信息

---

## CMakeLists.txt 详解

### 文件结构

```cmake
cmake_minimum_required(VERSION 3.10)    # CMake最低版本
project(vehicle_localization C)         # 项目名称和语言

set(CMAKE_C_STANDARD 99)                # C标准版本
set(CMAKE_C_STANDARD_REQUIRED ON)       # 强制使用指定标准

set(CMAKE_C_FLAGS "...")                # 编译器标志
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ...) # 输出目录

add_library(...)                        # 创建库
add_executable(...)                     # 创建可执行文件
target_link_libraries(...)              # 链接库

install(...)                            # 安装配置
```

### 关键配置

#### 1. 编译标志

```cmake
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")
```

- `-Wall`: 启用所有常见警告
- `-Wextra`: 启用额外警告
- `-O2`: 优化级别2（Release模式）
- 可根据需要修改

#### 2. 库的创建

```cmake
add_library(localization_lib ${SOURCES})
target_link_libraries(localization_lib PRIVATE m)  # 链接数学库
```

**优点**：
- 可供其他项目链接
- 代码复用性高
- 便于单元测试

#### 3. 可执行文件

```cmake
add_executable(localization_example example_usage.c)
target_link_libraries(localization_example PRIVATE localization_lib m)
```

**依赖**：
- 链接 `localization_lib` 库
- 链接数学库 `m`

#### 4. 安装配置

```cmake
install(TARGETS localization_example RUNTIME DESTINATION bin)
install(FILES ... DESTINATION include)
```

**用途**：
- 使程序可系统范围内使用
- 提供头文件供外部使用

---

## 构建类型选择

### Release（生产环境）
```bash
./cmake_build.sh build
```

**特点**：
- 启用优化（-O2）
- 无调试符号
- 文件大小小
- 执行速度快
- ✓ 推荐用于部署

### Debug（开发调试）
```bash
./cmake_build.sh build-debug
```

**特点**：
- 包含调试符号（-g）
- 无优化
- 文件大小大
- 执行速度慢
- ✓ 推荐用于开发

---

## 编译输出说明

### 成功编译示例

```
ℹ 开始构建项目 (Release模式)...
ℹ 运行CMake...
-- The C compiler identification is AppleClang ...
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- ========================================
-- Vehicle Localization System
-- Version: 1.0.0
-- C Standard: C99
-- Build Type: Release
-- Compiler: /usr/bin/cc
-- ========================================
-- Compiler Flags: -Wall -Wextra -O2
-- Output Directory: /path/to/build/bin

[ 25%] Linking C executable bin/localization_example
[100%] Built target localization_example
✓ 构建完成！
```

### 常见问题

#### 问题1：找不到CMake
**症状**：`cmake: command not found`

**解决**：
```bash
# macOS (使用Homebrew)
brew install cmake

# Ubuntu/Debian
sudo apt-get install cmake

# 验证安装
cmake --version
```

#### 问题2：编译失败
**症状**：`fatal error: ... No such file or directory`

**解决**：
```bash
# 清除并重新构建
./cmake_build.sh rebuild

# 检查源文件是否完整
ls -la *.c *.h
```

#### 问题3：版本不兼容
**症状**：`CMake 3.10 or higher is required`

**解决**：升级CMake版本

```bash
# macOS
brew upgrade cmake

# Ubuntu
sudo apt-get update && sudo apt-get upgrade cmake
```

---

## 与Makefile的对比

| 特性 | CMake | Makefile |
|------|-------|----------|
| **跨平台** | ✓ 优秀 | △ 需要修改 |
| **易用性** | ✓ 更易学 | △ 较复杂 |
| **可维护性** | ✓ 高 | △ 中 |
| **IDE支持** | ✓ 完整 | △ 有限 |
| **性能** | ✓ 相同 | ✓ 相同 |

**建议**：
- 新项目：使用CMake
- 维护旧项目：可保留Makefile
- 并行使用：两者共存不冲突

---

## 进阶用法

### 1. 自定义编译选项

编辑 CMakeLists.txt，修改编译标志：

```cmake
# 添加更多警告
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic -Wconversion")

# 启用LTO优化（仅支持GCC 4.9+）
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -flto")

# 添加特定平台选项
if(APPLE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
endif()
```

### 2. 创建子目录结构

```cmake
# CMakeLists.txt (根目录)
add_subdirectory(src)
add_subdirectory(tests)

# src/CMakeLists.txt
add_library(mylib ...)

# tests/CMakeLists.txt
add_executable(test_program ...)
target_link_libraries(test_program mylib)
```

### 3. 条件编译

```cmake
# Debug模式下添加额外定义
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(ENABLE_LOGGING)
endif()

# 平台相关编译
if(APPLE)
    target_compile_definitions(myapp PRIVATE USE_MACOS_API)
elseif(WIN32)
    target_compile_definitions(myapp PRIVATE USE_WINDOWS_API)
endif()
```

### 4. 外部库链接

```cmake
# 查找系统库
find_package(OpenSSL REQUIRED)
target_link_libraries(myapp PRIVATE OpenSSL::SSL)

# 手动指定库路径
link_directories(/usr/local/lib)
target_link_libraries(myapp PRIVATE mylib)
```

---

## 性能优化

### 编译时间优化

```cmake
# 启用并行编译（在脚本中）
make -j$(nproc)

# 或在CMakeLists.txt中设置
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
```

### 运行时性能优化

```cmake
# 启用Link-Time Optimization (LTO)
if(POLICY CMP0069)
    cmake_policy(SET CMP0069 NEW)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()

# 启用优化级别3
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
```

---

## 故障排查

### 清除CMake缓存

```bash
# 删除build目录的所有CMake缓存
rm -rf build/CMakeFiles build/CMakeCache.txt

# 或使用脚本
./cmake_build.sh clean && ./cmake_build.sh build
```

### 检查编译细节

```bash
# 显示完整的编译命令
cd build && make VERBOSE=1

# 或
cd build && cmake --build . --verbose
```

### 验证编译器

```bash
# 检查使用的编译器
cc --version
gcc --version

# 在CMakeLists.txt中查看
message(STATUS "Compiler: ${CMAKE_C_COMPILER}")
```

---

## 最佳实践

1. **始终使用out-of-source构建**
   ```bash
   mkdir build && cd build
   cmake ..
   ```

2. **定期清除和重建**
   ```bash
   ./cmake_build.sh rebuild
   ```

3. **为不同用途创建不同build**
   ```bash
   mkdir build-release build-debug
   cd build-release && cmake -DCMAKE_BUILD_TYPE=Release ..
   cd build-debug && cmake -DCMAKE_BUILD_TYPE=Debug ..
   ```

4. **在版本控制中忽略build目录**
   ```
   # .gitignore
   build/
   CMakeUserPresets.json
   ```

5. **文档化CMake选项**
   ```cmake
   option(ENABLE_TESTS "Enable test builds" ON)
   option(ENABLE_SHARED_LIBS "Build shared libraries" OFF)
   ```

---

## 总结

| 场景 | 命令 |
|------|------|
| 第一次编译 | `./cmake_build.sh build` |
| 开发调试 | `./cmake_build.sh build-debug && ./cmake_build.sh run` |
| 清除和重建 | `./cmake_build.sh rebuild` |
| 生产部署 | `./cmake_build.sh build && ./cmake_build.sh install` |
| 查看帮助 | `./cmake_build.sh help` |

**推荐工作流**：
1. `./cmake_build.sh build` - 初始构建
2. 编辑源代码
3. `./cmake_build.sh run` - 测试运行
4. 重复2-3直到完成
5. `./cmake_build.sh rebuild` - 最终验证

---

**文档版本**：1.0  
**最后更新**：2025年12月16日
