   #ifndef LOCALIZATION_MACRTOS_H
   #define LOCALIZATION_MACRTOS_H
   /* ========== 常数定义 ========== */

#define PI 3.141592653589793
#define DEG_TO_RAD(x) ((x) * PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / PI)
#define WHEELBASE 2.7  /* 轴距 (m) */
#define MAX_HISTORY_SIZE 100

// /* 速度阈值 */
#define V_STATIC_THRESHOLD 0.05 /* 静止速度阈值 (m/s), 优化为float精度推荐值 */
#define V_LOW_SPEED_THRESHOLD 1.5  /* 低速阈值 (m/s) */
#define V_PARKING_MAX 2.0  /* 自动泊车最大速度 (m/s) */

/* 传感器阈值 */
#define WHEEL_SLIP_ACCEL_THRESHOLD 0.3  /* 轮滑动加速度阈值 (m/s²) */
#endif