# CMake 快速参考卡

## 🚀 一行命令快速开始

```bash
cd /Users/alex/workspace/code_review/vehicle_localization && \
./cmake_build.sh build && ./cmake_build.sh run
```

**耗时**：10秒  
**结果**：看到程序运行输出 ✓

---

## 📌 8个核心命令

| 命令 | 功能 | 耗时 |
|------|------|------|
| `./cmake_build.sh build` | 编译（Release） | 5-10秒 |
| `./cmake_build.sh build-debug` | 编译（Debug） | 5-10秒 |
| `./cmake_build.sh run` | 运行程序 | <5秒 |
| `./cmake_build.sh clean` | 清除编译产物 | <1秒 |
| `./cmake_build.sh rebuild` | 清除后重新编译 | 5-10秒 |
| `./cmake_build.sh install` | 安装到系统 | <5秒 |
| `./cmake_build.sh test` | 运行测试 | <5秒 |
| `./cmake_build.sh help` | 显示帮助 | <1秒 |

---

## 📂 文件位置

| 类型 | 位置 |
|------|------|
| 可执行文件 | `build/bin/localization_example` |
| 静态库 | `build/lib/liblocalization_lib.a` |
| CMake配置 | `CMakeLists.txt` |
| 构建脚本 | `cmake_build.sh` |
| 构建缓存 | `build/CMakeCache.txt` |

---

## 🔧 常见用途

### 日常开发

```bash
./cmake_build.sh build      # 编译
./cmake_build.sh run        # 运行
./cmake_build.sh clean      # 清除
```

### 调试

```bash
./cmake_build.sh build-debug    # Debug编译
gdb ./build/bin/localization_example
```

### 完全重建

```bash
./cmake_build.sh rebuild    # 等同于 clean + build
```

### 跨IDE使用

```bash
# VS Code: 自动识别CMakeLists.txt
# CLion: 自动识别CMakeLists.txt  
# Visual Studio: File → Open → CMakeLists.txt
```

---

## 📚 文档速查

| 需求 | 文档 |
|------|------|
| **快速开始** | CMAKE_STRUCTURE.md |
| **详细指南** | CMAKE_BUILD_GUIDE.md |
| **工具对比** | CMAKE_vs_MAKEFILE.md |
| **问题诊断** | CMAKE_CHECKLIST.md |
| **总体概览** | CMAKE_OVERVIEW.md（本文件） |

---

## ⚡ 关键特性

```
✓ 跨平台支持        (Windows/macOS/Linux)
✓ IDE完整集成      (VS Code/CLion/VS)
✓ Debug/Release模式 (一键切换)
✓ 自动并行编译     (充分利用多核)
✓ 库+可执行文件管理 (分离构建)
✓ 自动依赖检测     (无需手动配置)
✓ 代码输出隔离     (source/build分离)
```

---

## 🎯 编译输出说明

### 成功编译（Release）

```
✓ 构建完成！
ℹ 可执行文件位置: /path/to/build/bin/localization_example
```

### Debug编译

```
编译时添加 -g 调试符号，文件大小会增大
可直接用 gdb 调试
```

### 增量编译

```
只重新编译改动的文件（<1秒）
如果改动CMakeLists.txt需要完全重建
```

---

## 🐛 快速故障排查

| 症状 | 原因 | 解决 |
|------|------|------|
| `cmake: command not found` | CMake未安装 | `brew install cmake` |
| 编译失败 | 文件不完整或损坏 | `./cmake_build.sh rebuild` |
| 可执行文件不存在 | 未成功编译 | `./cmake_build.sh build` |
| 权限拒绝 | 脚本无执行权限 | `chmod +x cmake_build.sh` |

---

## 📊 系统要求

| 项目 | 版本/要求 |
|------|----------|
| CMake | 3.10+ |
| C编译器 | GCC/Clang |
| C标准 | C99 |
| 操作系统 | macOS 10.13+, Linux, Windows |

---

## 🔄 完整工作流程

```
1. 编译代码
   ./cmake_build.sh build

2. 运行程序
   ./cmake_build.sh run

3. 编辑源代码
   vim localization_system.c

4. 重新编译
   ./cmake_build.sh build

5. 测试运行
   ./cmake_build.sh run

6. 调试问题（如需）
   ./cmake_build.sh build-debug
   gdb ./build/bin/localization_example

7. 清除临时文件
   ./cmake_build.sh clean

8. 最终验证
   ./cmake_build.sh rebuild && ./cmake_build.sh run
```

---

## 💡 关键概念

### CMake 是什么？

CMake 是一个跨平台的编译系统生成器。它读取 CMakeLists.txt，生成各平台的编译脚本（Makefile、Visual Studio项目等）。

### CMake 相比 Makefile

| 特性 | CMake | Makefile |
|------|-------|----------|
| 跨平台 | ✓ 自动 | △ 需要修改 |
| 易用性 | ✓ 更易学 | △ 较复杂 |
| IDE支持 | ✓ 完整 | △ 有限 |

### 为什么要用CMake？

1. **跨平台**：同一套配置适用所有系统
2. **现代**：符合现代项目实践
3. **灵活**：易于扩展和维护
4. **友好**：IDE完整支持

---

## 🎓 学习资源

### 官方资源
- [CMake官方网站](https://cmake.org/)
- [CMake官方教程](https://cmake.org/cmake/help/latest/guide/tutorial/)

### 项目文档
- 详细指南：[CMAKE_BUILD_GUIDE.md](CMAKE_BUILD_GUIDE.md)
- 结构说明：[CMAKE_STRUCTURE.md](CMAKE_STRUCTURE.md)
- 工具对比：[CMAKE_vs_MAKEFILE.md](CMAKE_vs_MAKEFILE.md)

---

## 🚨 常见错误

### 错误1：cmake not found

```bash
# macOS
brew install cmake

# Ubuntu
sudo apt-get install cmake

# 验证
cmake --version
```

### 错误2：编译失败 with 奇怪的错误

```bash
# 解决方法：完全重建
./cmake_build.sh rebuild
```

### 错误3：找不到可执行文件

```bash
# 检查是否编译成功
ls -l build/bin/localization_example

# 如果不存在，编译
./cmake_build.sh build
```

---

## ⚙️ 高级选项

### 手动CMake命令（不推荐，用脚本更简单）

```bash
mkdir build && cd build

# Release编译
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8

# Debug编译  
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j8

# 运行
./bin/localization_example
```

### 更激进的优化（仅Release）

编辑 CMakeLists.txt：
```cmake
set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native -flto")
```

---

## 📞 获取帮助

### 脚本内置帮助
```bash
./cmake_build.sh help
```

### 查看脚本内容
```bash
cat cmake_build.sh
```

### 查看编译详情
```bash
cd build && make VERBOSE=1
```

---

## 🎯 总结

| 任务 | 命令 |
|------|------|
| 首次使用 | `./cmake_build.sh build && ./cmake_build.sh run` |
| 日常开发 | `./cmake_build.sh build` + 编辑代码 + `./cmake_build.sh run` |
| 调试问题 | `./cmake_build.sh build-debug` + `gdb ./build/bin/localization_example` |
| 清除垃圾 | `./cmake_build.sh clean` |
| 完全重建 | `./cmake_build.sh rebuild` |
| 获取帮助 | `./cmake_build.sh help` |

---

## 🎉 结论

✅ **CMake工程已完整部署**  
✅ **可立即使用**  
✅ **文档齐全**  
✅ **跨平台支持**  

**开始使用**：`./cmake_build.sh build && ./cmake_build.sh run`

---

**版本**: 1.0  
**日期**: 2025年12月16日  
**状态**: ✅ 经过验证
