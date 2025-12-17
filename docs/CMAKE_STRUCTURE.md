# CMake 工程结构说明

本项目使用 **CMake** 作为现代构建系统，支持跨平台编译和IDE集成。

## 📁 工程文件结构

```
vehicle_localization/
├── CMakeLists.txt                          # ⭐ CMake配置文件
├── cmake_build.sh                          # ⭐ 快速构建脚本
├── CMAKE_BUILD_GUIDE.md                    # ⭐ CMake使用指南
│
├── Makefile                                # 传统Makefile（保留）
├── build_and_run.sh                        # 传统构建脚本
│
├── 源代码文件
│   ├── motion_model.c/h                   # 运动模型模块
│   ├── kalman_filter.c/h                  # 卡尔曼滤波模块
│   ├── rtk_calibration.c/h                # RTK标定模块
│   ├── localization_system.c/h            # 定位系统主模块
│   ├── localization_system_private.h      # 私有头文件
│   ├── localization_types.h               # 类型定义
│   └── example_usage.c                    # 示例程序
│
├── build/                                  # ⭐ 编译输出目录（自动生成）
│   ├── bin/
│   │   └── localization_example           # 可执行文件
│   ├── lib/
│   │   └── liblocalization_lib.a          # 静态库
│   ├── CMakeFiles/                        # CMake缓存
│   └── CMakeCache.txt                     # CMake缓存配置
│
├── 文档文件
│   ├── README.md                          # 项目说明
│   ├── PARAMETER_CALIBRATION_GUID.md      # 参数标定指南（2500行）
│   ├── C_IMPLEMENTATION_GUIDE.md          # C实现指南
│   ├── INTEGRATION_GUIDE.md               # 集成指南
│   ├── FILE_MANIFEST.md                   # 文件清单
│   ├── PROJECT_COMPLETION_SUMMARY.md      # 项目完成总结
│   ├── low_speed_localization_guide.md    # 低速定位指南
│   └── CMAKE_BUILD_GUIDE.md               # CMake构建指南（本文件）
│
├── .git/                                   # Git版本控制
├── .gitignore                              # 忽略规则
└── localization_system.py                  # Python参考实现

```

## 🚀 快速开始

### 最简单的方式（推荐）

```bash
# 进入项目目录
cd /Users/alex/workspace/code_review/vehicle_localization

# 一键构建
./cmake_build.sh build

# 运行程序
./cmake_build.sh run

# 清除编译产物
./cmake_build.sh clean
```

### 输出文件位置

编译成功后：
- 📦 **可执行文件**: `build/bin/localization_example`
- 📚 **静态库**: `build/lib/liblocalization_lib.a`
- 🔧 **编译信息**: `build/CMakeCache.txt`

## 📋 CMakeLists.txt 详解

### 核心部分

```cmake
# 1. 项目声明
cmake_minimum_required(VERSION 3.10)
project(vehicle_localization C)

# 2. C标准设置
set(CMAKE_C_STANDARD 99)

# 3. 编译选项
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -O2")

# 4. 输出目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 5. 源文件列表
set(SOURCES
    motion_model.c
    kalman_filter.c
    rtk_calibration.c
    localization_system.c
)

# 6. 创建库
add_library(localization_lib ${SOURCES})
target_link_libraries(localization_lib PRIVATE m)

# 7. 创建可执行文件
add_executable(localization_example example_usage.c)
target_link_libraries(localization_example PRIVATE localization_lib m)

# 8. 安装配置
install(TARGETS localization_example RUNTIME DESTINATION bin)
install(TARGETS localization_lib LIBRARY DESTINATION lib)
install(FILES *.h DESTINATION include)
```

### 关键特性

| 特性 | 说明 |
|------|------|
| **库分离** | 源代码编译为 `localization_lib` 库 |
| **可执行文件** | `localization_example` 链接库并执行 |
| **数学库** | 自动链接 `-lm`（POSIX系统） |
| **Debug模式** | 可选编译调试符号（`-g`） |
| **安装支持** | 支持 `make install` 安装到系统 |

## 🔧 构建脚本命令

### 脚本功能汇总

| 命令 | 功能 | 耗时 |
|------|------|------|
| `build` | Release构建 | 5-10秒 |
| `build-debug` | Debug构建 | 5-10秒 |
| `clean` | 清除编译产物 | <1秒 |
| `rebuild` | 清除后重新构建 | 5-10秒 |
| `run` | 运行可执行文件 | <5秒 |
| `install` | 安装到系统 | <5秒 |
| `test` | 运行测试 | <5秒 |
| `help` | 显示帮助 | <1秒 |

### 脚本源码位置

**文件**: `cmake_build.sh`（可执行权限 755）

**关键函数**:
- `build_project()` - CMake构建逻辑
- `clean_build()` - 清除编译产物
- `run_example()` - 运行程序
- 彩色输出支持（✓ ✗ ⚠ ℹ）

## 📊 编译过程详解

### Step 1: CMake配置

```
ℹ 运行CMake...
-- The C compiler identification is AppleClang 17.0.0
-- Detecting C compiler ABI info
-- ========================================
-- Vehicle Localization System
-- Version: 1.0.0
-- C Standard: C99
-- Build Type: Release
-- ========================================
```

**作用**: 检测编译器、配置环境、生成Makefile

### Step 2: 编译源文件

```
[ 25%] Building C object CMakeFiles/localization_lib.dir/motion_model.c.o
[ 50%] Building C object CMakeFiles/localization_lib.dir/kalman_filter.c.o
[ 75%] Building C object CMakeFiles/localization_lib.dir/localization_system.c.o
```

**编译顺序**:
1. motion_model.c
2. kalman_filter.c
3. rtk_calibration.c
4. localization_system.c

### Step 3: 链接库

```
[ 85%] Linking C static library lib/liblocalization_lib.a
```

**操作**: 将所有.o文件合并成静态库

### Step 4: 链接可执行文件

```
[100%] Linking C executable bin/localization_example
```

**操作**: 将example_usage.c与localization_lib链接

## 🎯 与Makefile的关系

### 保留Makefile的原因

✓ 向后兼容  
✓ 简单快速编译 (`make`)  
✓ 学习参考  

### 优先使用CMake的原因

✓ 跨平台支持  
✓ IDE友好  
✓ 库管理更清晰  
✓ 扩展性更好  

### 并行使用

两套系统**互不影响**：
- Makefile 生成 `localization_example`
- CMake 生成 `build/bin/localization_example`

```bash
make                    # 使用Makefile
./cmake_build.sh build  # 使用CMake
```

## 🛠️ IDE集成

### VS Code

1. 安装扩展：`CMake Tools`
2. 打开项目文件夹
3. 自动识别 CMakeLists.txt
4. 按 F7 构建，F5 调试

### CLion

1. 打开项目
2. 自动识别 CMakeLists.txt
3. 选择工具链（Xcode/GCC）
4. 构建和运行

### Visual Studio (Windows)

1. 打开项目文件夹
2. Visual Studio 自动识别 CMake
3. 选择编译配置
4. Build → Build All

## 📦 编译产物说明

### build/bin/localization_example

**类型**: 可执行文件  
**大小**: ~50-100 KB  
**依赖**: libc, libm  
**运行**: `./build/bin/localization_example`

### build/lib/liblocalization_lib.a

**类型**: 静态库  
**大小**: ~50-100 KB  
**用途**: 供其他项目链接  
**用法**:
```c
#include "localization_system.h"
// 链接 liblocalization_lib.a
```

## 🔍 常见问题

### Q: 为什么有两套构建系统？
A: 兼容性。Makefile让现有用户满意，CMake支持未来扩展。

### Q: 应该用哪个？
A: **CMake**（新用户）或 **Makefile**（旧用户）。不用两个。

### Q: 如何只用CMake删除Makefile？
A: 可以，但建议保留以支持make用户。

### Q: 编译失败怎么办？
A: 运行 `./cmake_build.sh rebuild` 强制重新编译。

### Q: 为什么编译很慢？
A: 第一次需要CMake配置，后续会快。可用 `./cmake_build.sh run` 避免重复配置。

## 📈 性能优化

### 并行编译

脚本自动使用系统CPU核心数（Mac/Linux）：
```bash
make -j$(nproc)  # 默认
make -j4         # 指定4核
```

### 优化编译器标志

编辑CMakeLists.txt：
```cmake
set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native")  # 更激进优化
```

## 📚 相关文档

| 文档 | 内容 |
|------|------|
| **CMAKE_BUILD_GUIDE.md** | 详细的CMake使用指南 |
| **C_IMPLEMENTATION_GUIDE.md** | C代码实现细节 |
| **PARAMETER_CALIBRATION_GUID.md** | 参数标定手册（2500行）|
| **INTEGRATION_GUIDE.md** | 集成到其他项目 |

## 🎓 学习资源

### CMake官方
- [CMake Documentation](https://cmake.org/cmake/help/latest/)
- [CMake Tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)

### 本项目
- 查看 CMakeLists.txt（注释清晰）
- 查看 CMAKE_BUILD_GUIDE.md（完整指南）
- 运行 `./cmake_build.sh help`（快速参考）

## 🚀 下一步

1. ✅ 阅读本文件了解结构
2. ✅ 运行 `./cmake_build.sh build` 编译
3. ✅ 运行 `./cmake_build.sh run` 测试
4. ✅ 参考 CMAKE_BUILD_GUIDE.md 深入学习
5. ✅ 修改代码并重新编译测试

---

**版本**: 1.0  
**最后更新**: 2025年12月16日  
**CMake版本**: 3.10+  
**C标准**: C99
