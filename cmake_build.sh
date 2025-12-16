#!/bin/bash

# 车辆定位系统 CMake 快速构建脚本

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # 无颜色

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
SOURCE_DIR="${SCRIPT_DIR}"

# 打印帮助信息
print_help() {
    echo -e "${BLUE}=== Vehicle Localization System - Build Script ===${NC}"
    echo ""
    echo "用法: $0 [command]"
    echo ""
    echo "命令:"
    echo "  build        - 构建项目（Release模式）"
    echo "  build-debug  - 构建项目（Debug模式）"
    echo "  clean        - 清除编译产物"
    echo "  rebuild      - 清除并重新构建"
    echo "  install      - 安装编译结果"
    echo "  run          - 运行可执行文件"
    echo "  test         - 运行所有测试"
    echo "  help         - 显示此帮助信息"
    echo ""
}

# 打印信息
print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# 打印成功
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# 打印警告
print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# 打印错误
print_error() {
    echo -e "${RED}✗ $1${NC}"
}

# 构建函数
build_project() {
    local build_type=$1
    
    print_info "开始构建项目 (${build_type}模式)..."
    
    # 创建构建目录
    mkdir -p "${BUILD_DIR}"
    cd "${BUILD_DIR}"
    
    # 运行CMake
    print_info "运行CMake..."
    cmake -DCMAKE_BUILD_TYPE="${build_type}" "${SOURCE_DIR}"
    
    # 编译
    print_info "编译项目..."
    # 获取CPU核心数（兼容 macOS 和 Linux）
    if command -v nproc &> /dev/null; then
        JOBS=$(nproc)
    elif command -v sysctl &> /dev/null; then
        JOBS=$(sysctl -n hw.logicalcpu)
    else
        JOBS=1
    fi
    make -j${JOBS}
    
    print_success "构建完成！"
    print_info "可执行文件位置: ${BUILD_DIR}/bin/localization_example"
}

# 清除函数
clean_build() {
    if [ -d "${BUILD_DIR}" ]; then
        print_info "删除构建目录..."
        rm -rf "${BUILD_DIR}"
        print_success "清除完成"
    else
        print_warning "构建目录不存在"
    fi
}

# 运行函数
run_example() {
    if [ ! -f "${BUILD_DIR}/bin/localization_example" ]; then
        print_error "可执行文件不存在，请先运行 build 命令"
        exit 1
    fi
    
    print_info "运行示例..."
    "${BUILD_DIR}/bin/localization_example"
}

# 安装函数
install_project() {
    if [ ! -d "${BUILD_DIR}" ]; then
        print_error "构建目录不存在，请先运行 build 命令"
        exit 1
    fi
    
    cd "${BUILD_DIR}"
    print_info "安装编译结果..."
    make install
    print_success "安装完成"
}

# 测试函数
run_tests() {
    if [ ! -f "${BUILD_DIR}/bin/localization_example" ]; then
        print_error "可执行文件不存在，请先运行 build 命令"
        exit 1
    fi
    
    print_info "运行测试..."
    "${BUILD_DIR}/bin/localization_example"
}

# 主程序
main() {
    local command=${1:-help}
    
    case "${command}" in
        build)
            build_project "Release"
            ;;
        build-debug)
            build_project "Debug"
            ;;
        clean)
            clean_build
            ;;
        rebuild)
            clean_build
            build_project "Release"
            ;;
        install)
            install_project
            ;;
        run)
            run_example
            ;;
        test)
            run_tests
            ;;
        help|*)
            print_help
            ;;
    esac
}

# 运行主程序
main "$@"
