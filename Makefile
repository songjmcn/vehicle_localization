CC = gcc
CFLAGS = -Wall -Wextra -O2 -std=c99 -lm
LDFLAGS = -lm

# 源文件
SOURCES = motion_model.c kalman_filter.c rtk_calibration.c localization_system.c wheel_odometry_processor.c example_usage.c
OBJECTS = $(SOURCES:.c=.o)
TARGET = localization_example

# 头文件
HEADERS = localization_types.h motion_model.h kalman_filter.h rtk_calibration.h localization_system.h wheel_odometry_processor.h

# 编译目标
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	@echo "✓ Build successful: $@"

%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "✓ Clean successful"

run: $(TARGET)
	./$(TARGET)

# 编译单个模块（用于测试）
test_motion: motion_model.o localization_types.h
	@echo "✓ Motion model module compiled"

test_filter: kalman_filter.o motion_model.o localization_types.h
	@echo "✓ Kalman filter module compiled"

test_rtk: rtk_calibration.o localization_types.h
	@echo "✓ RTK calibration module compiled"

.PHONY: all clean run test_motion test_filter test_rtk
