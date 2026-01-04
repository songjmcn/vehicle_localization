CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -lm -Isrc -Itools
LDFLAGS = -lm

# 源文件
SOURCES = src/motion_model.c src/kalman_filter.c src/steering_model.c src/wheel_odometry_processor.c src/steering_gain_calibration.c tools/rtk_calibration.c src/localization_system.c example_usage.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = localization_example

# 头文件
HEADERS = src/localization_types.h src/motion_model.h src/kalman_filter.h src/steering_model.h src/wheel_odometry_processor.h src/steering_gain_calibration.h tools/rtk_calibration.h src/localization_system.h

# 编译目标
all: $(TARGET)

$(TARGET): $(OBJECTS)
	./$(TARGET)

# 编译单个模块（用于测试）
test_motion: src/motion_model.o src/localization_types.h
	@echo "✓ Motion model module compiled"

test_filter: src/kalman_filter.o src/motion_model.o src/localization_types.h
	@echo "✓ Kalman filter module compiled"

test_rtk: tools/rtk_calibration.o src/localization_types.h
	@echo "✓ RTK calibration module compiled"

.PHONY: all clean run test_motion test_filter test_rtk
