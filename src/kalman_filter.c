/**
 * @file kalman_filter.c
 * @brief 扩展卡尔曼滤波器实现
 * 
 * 本文件实现方案 B（混合精度）的卡尔曼滤波器：
 * - 所有矩阵操作使用内部 double 精度版本
 * - Float API 通过适配层自动转换
 */

#include "kalman_filter.h"
#include "motion_model.h"
#include "localization_adapter.h"
#include "localization_types_internal.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* ========== 矩阵操作函数 ========== */

/**
 * @brief 矩阵乘法 C = A * B (n x m) * (m x p)
 */
// static void matrix_multiply(double *A, double *B, double *C,
//                            int n, int m, int p) {
//     memset(C, 0, n * p * sizeof(double));
//
//     for (int i = 0; i < n; i++) {
//         for (int j = 0; j < p; j++) {
//             for (int k = 0; k < m; k++) {
//                 C[i * p + j] += A[i * m + k] * B[k * p + j];
//             }
//         }
//     }
// }

/**
 * @brief 矩阵转置 B = A^T (n x m -> m x n)
 */
// static void matrix_transpose(double *A, double *B, int n, int m) {
//     for (int i = 0; i < n; i++) {
//         for (int j = 0; j < m; j++) {
//             B[j * n + i] = A[i * m + j];
//         }
//     }
// }

/**
 * @brief 矩阵加法 C = A + B (n x m)
 */
// static void matrix_add(double *A, double *B, double *C, int n, int m) {
//     for (int i = 0; i < n * m; i++) {
//         C[i] = A[i] + B[i];
//     }
// }

/**
 * @brief 矩阵减法 C = A - B (n x m)
 */
// static void matrix_subtract(double *A, double *B, double *C, int n, int m) {
//     for (int i = 0; i < n * m; i++) {
//         C[i] = A[i] - B[i];
//     }
// }

/**
 * @brief 矩阵求逆 (仅支持3x3和5x5)
 * @return 0成功，-1失败
 */
static int matrix_inverse_3x3(double *A, double *A_inv) {
    /* 计算行列式 */
    double det = A[0] * (A[4] * A[8] - A[5] * A[7])
               - A[1] * (A[3] * A[8] - A[5] * A[6])
               + A[2] * (A[3] * A[7] - A[4] * A[6]);
    
    if (fabs(det) < 1e-10) {
        return -1;  /* 矩阵奇异 */
    }
    
    double inv_det = 1.0 / det;
    
    A_inv[0] = (A[4] * A[8] - A[5] * A[7]) * inv_det;
    A_inv[1] = (A[2] * A[7] - A[1] * A[8]) * inv_det;
    A_inv[2] = (A[1] * A[5] - A[2] * A[4]) * inv_det;
    
    A_inv[3] = (A[5] * A[6] - A[3] * A[8]) * inv_det;
    A_inv[4] = (A[0] * A[8] - A[2] * A[6]) * inv_det;
    A_inv[5] = (A[2] * A[3] - A[0] * A[5]) * inv_det;
    
    A_inv[6] = (A[3] * A[7] - A[4] * A[6]) * inv_det;
    A_inv[7] = (A[1] * A[6] - A[0] * A[7]) * inv_det;
    A_inv[8] = (A[0] * A[4] - A[1] * A[3]) * inv_det;
    
    return 0;
}

/**
 * @brief 5x5矩阵求逆（高斯消元法）
 */
// static int matrix_inverse_5x5(double A[5][5], double A_inv[5][5]) {
//     double temp[5][10];
//
//     /* 构造增广矩阵 [A | I] */
//     for (int i = 0; i < 5; i++) {
//         for (int j = 0; j < 5; j++) {
//             temp[i][j] = A[i][j];
//             temp[i][j + 5] = (i == j) ? 1.0 : 0.0;
//         }
//     }
//
//     /* 高斯消元 */
//     for (int i = 0; i < 5; i++) {
//         /* 找主元 */
//         int pivot = i;
//         for (int k = i + 1; k < 5; k++) {
//             if (fabs(temp[k][i]) > fabs(temp[pivot][i])) {
//                 pivot = k;
//             }
//         }
//
//         if (fabs(temp[pivot][i]) < 1e-10) {
//             return -1;  /* 矩阵奇异 */
//         }
//
//         /* 行交换 */
//         if (pivot != i) {
//             for (int j = 0; j < 10; j++) {
//                 double t = temp[i][j];
//                 temp[i][j] = temp[pivot][j];
//                 temp[pivot][j] = t;
//             }
//         }
//
//         /* 主元化为1 */
//         double div = temp[i][i];
//         for (int j = 0; j < 10; j++) {
//             temp[i][j] /= div;
//         }
//
//         /* 消元 */
//         for (int k = 0; k < 5; k++) {
//             if (k != i) {
//                 double mul = temp[k][i];
//                 for (int j = 0; j < 10; j++) {
//                     temp[k][j] -= mul * temp[i][j];
//                 }
//             }
//         }
//     }
//
//     /* 提取逆矩阵 */
//     for (int i = 0; i < 5; i++) {
//         for (int j = 0; j < 5; j++) {
//             A_inv[i][j] = temp[i][j + 5];
//         }
//     }
//
//     return 0;
// }

/**
 * @brief 4x4矩阵求逆（高斯消元法）(新增)
 */
static int matrix_inverse_4x4(double A[4][4], double A_inv[4][4]) {
    double temp[4][8];
    
    /* 构造增广矩阵 [A | I] */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp[i][j] = A[i][j];
            temp[i][j + 4] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    /* 高斯消元 */
    for (int i = 0; i < 4; i++) {
        /* 找主元 */
        int pivot = i;
        for (int k = i + 1; k < 4; k++) {
            if (fabs(temp[k][i]) > fabs(temp[pivot][i])) {
                pivot = k;
            }
        }
        
        if (fabs(temp[pivot][i]) < 1e-10) {
            return -1;  /* 矩阵奇异 */
        }
        
        /* 行交换 */
        if (pivot != i) {
            for (int j = 0; j < 8; j++) {
                double t = temp[i][j];
                temp[i][j] = temp[pivot][j];
                temp[pivot][j] = t;
            }
        }
        
        /* 主元化为1 */
        double div = temp[i][i];
        for (int j = 0; j < 8; j++) {
            temp[i][j] /= div;
        }
        
        /* 消元 */
        for (int k = 0; k < 4; k++) {
            if (k != i) {
                double mul = temp[k][i];
                for (int j = 0; j < 8; j++) {
                    temp[k][j] -= mul * temp[i][j];
                }
            }
        }
    }
    
    /* 提取逆矩阵 */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            A_inv[i][j] = temp[i][j + 4];
        }
    }
    
    return 0;
}

/* ========== 卡尔曼滤波器函数 ========== */

int kalman_filter_init(KalmanFilter_t *kf,
                       const VehicleState_t *initial_state,
                       const NoiseParameters_t *noise,
                       const CalibrationParams_t *calibration) {
    if (!kf || !initial_state || !noise || !calibration) {
        return -1;
    }
    
    /* 将 float 外部 API 转换为内部 double 精度 */
    VehicleState_internal_t state_internal;
    NoiseParameters_internal_t noise_internal;
    CalibrationParams_internal_t calib_internal;
    
    vehicle_state_float_to_internal(initial_state, &state_internal);
    noise_parameters_float_to_internal(noise, &noise_internal);
    calibration_params_float_to_internal(calibration, &calib_internal);
    
    /* 初始化状态 (Double 精度存储) */
    kf->state.x_hat[0] = state_internal.x;
    kf->state.x_hat[1] = state_internal.y;
    kf->state.x_hat[2] = state_internal.theta;
    kf->state.x_hat[3] = state_internal.v;
    kf->state.x_hat[4] = state_internal.delta;
    
    /* 初始化协方差矩阵（合理的初始不确定性） */
    memset(kf->state.P, 0, sizeof(kf->state.P));
    kf->state.P[0][0] = 10.0;   /* x位置：10米初始不确定性 */
    kf->state.P[1][1] = 10.0;   /* y位置：10米初始不确定性 */
    kf->state.P[2][2] = 1.0;    /* heading：1弧度（约57度） */
    kf->state.P[3][3] = 0.5;    /* velocity：0.5 m/s */
    kf->state.P[4][4] = 0.2;    /* steering_angle：0.2弧度（约11度） */
    
    /* 保存噪声参数和校准参数 (转换为 double) */
    kf->noise = *noise;  /* 复制 float 版本 */
    kf->calibration = *calibration;  /* 复制 float 版本 */
    
    /* 初始化时间戳 */
    kf->state.previous_timestamp = 0;
    
    /* 初始化档位 - 从初始状态中获取 */
    kf->state.previous_gear = initial_state->gear;
    
    /* 初始化诊断信息 */
    memset(&kf->diagnostics, 0, sizeof(kf->diagnostics));
    
    /* 初始化融合配置为基础算法(新增) */
    kf->fusion_config.mode = FUSION_MODE_BASIC;
    kf->fusion_config.imu_steering_vel_threshold = 0.5f;
    kf->fusion_config.imu_steering_weight = 1.0f;
    kf->fusion_config.enable_adaptive_weighting = false;
    
    /* 初始化转向模型配置为NULL */
    kf->steering_config = NULL;
    
    return 0;
}

/**
 * @brief 以指定融合模式初始化卡尔曼滤波器(新增)
 */
int kalman_filter_init_with_mode(KalmanFilter_t *kf,
                                  const VehicleState_t *initial_state,
                                  const NoiseParameters_t *noise,
                                  const CalibrationParams_t *calibration,
                                  const FusionConfig_t *fusion_config) {
    /* 首先调用基础初始化 */
    int result = kalman_filter_init(kf, initial_state, noise, calibration);
    if (result != 0) {
        return result;
    }
    
    /* 设置融合配置(新增) */
    if (fusion_config) {
        kf->fusion_config = *fusion_config;
    }
    
    return 0;
}

/**
 * @brief 设置卡尔曼滤波器的转向模型配置
 */
int kalman_filter_set_steering_config(KalmanFilter_t *kf, SteeringModel_t *steering_config) {
    if (!kf) {
        return -1;
    }
    kf->steering_config = steering_config;
    return 0;
}

/**
 * @brief 预测步骤（带有运动方向）
 * 这个版本接收已计算的方向，避免在预测函数中重复计算方向
 */
void kalman_filter_predict_with_state(KalmanFilter_t *kf,
                                       const VehicleState_t *state_with_direction,
                                       double velocity,
                                       double steering_angle,
                                       uint64_t timestamp) {
    if (!kf || !state_with_direction || kf->state.previous_timestamp == 0) {
        kf->state.previous_timestamp = timestamp;
        return;
    }
    
    double dt = (double)(timestamp - kf->state.previous_timestamp) / 1000.0;
    if (dt <= 0 || dt > 1.0) {
        kf->state.previous_timestamp = timestamp;
        return;
    }
    
    /* 应用校准参数 */
    double cal_velocity = velocity * kf->calibration.wheel_calibration_factor;
    double cal_steering = steering_angle - kf->calibration.steering_angle_offset;
    cal_steering *= kf->calibration.steering_angle_gain;
    cal_steering = clamp(cal_steering, -(kf->steering_config->steering_config.max_steering_wheel_angle), kf->steering_config->steering_config.max_steering_wheel_angle);
    printf("velocity=%f, steering angle=%f, cal_velocity=%f, cal_steering=%f\n", velocity, steering_angle, cal_velocity, cal_steering);
    /* 当前状态（直接使用传入的状态，其中包含已计算的方向） */
    VehicleState_t current_state = *state_with_direction;
    current_state.steering_angle = kf->state.x_hat[4];

    /* 运动模型预测 */
    VehicleState_t predicted_state = motion_model_predict(kf->steering_config,
                                                           &current_state,
                                                           cal_velocity,
                                                           cal_steering,
                                                           dt);
    
    /* 更新状态 */
    kf->state.x_hat[0] = predicted_state.x;
    kf->state.x_hat[1] = predicted_state.y;
    kf->state.x_hat[2] = predicted_state.heading;
    kf->state.x_hat[3] = predicted_state.v;
    kf->state.x_hat[4] = predicted_state.steering_angle;
    kf->state.previous_gear = predicted_state.gear;

    /* 计算雅可比矩阵 */
    double F[5][5];
    memset(F, 0, sizeof(F));
    motion_model_jacobian(&current_state, cal_velocity, cal_steering, dt, F);

    /* 协方差矩阵的预测更新（P = F*P*F^T + Q） */
    double FP[5][5], FPF_T[5][5];

    /* 计算 F*P */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            FP[i][j] = 0.0;
            for (int k = 0; k < 5; k++) {
                FP[i][j] += F[i][k] * kf->state.P[k][j];
            }
        }
    }

    /* 计算 (F*P)*F^T */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            FPF_T[i][j] = 0.0;
            for (int k = 0; k < 5; k++) {
                FPF_T[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    /* P = F*P*F^T + Q */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            kf->state.P[i][j] = FPF_T[i][j];
        }
    }

    /* 加上过程噪声Q */
    kf->state.P[0][0] += kf->noise.q_x;
    kf->state.P[1][1] += kf->noise.q_y;
    kf->state.P[2][2] += kf->noise.q_theta;
    kf->state.P[3][3] += kf->noise.q_v;
    kf->state.P[4][4] += kf->noise.q_delta;

    kf->state.previous_timestamp = timestamp;
}

/**
 * @brief 预测步骤（原有版本，为了向后兼容）
 * 现在调用 kalman_filter_predict_with_state 的简化版本
 */
void kalman_filter_predict(KalmanFilter_t *kf,
                          double velocity,
                          double steering_angle,
                          uint64_t timestamp) {
    if (!kf) {
        return;
    }

    /* === 优化：简化版本 - 直接调用带状态的预测函数 === */
    /* 如果上层已经计算并传入方向，应使用 kalman_filter_predict_with_state */
    VehicleState_t current_state;
    current_state.x = kf->state.x_hat[0];
    current_state.y = kf->state.x_hat[1];
    current_state.heading = kf->state.x_hat[2];
    current_state.v = kf->state.x_hat[3];
    current_state.steering_angle = kf->state.x_hat[4];
    current_state.gear = kf->state.previous_gear;
    current_state.timestamp = timestamp;
    current_state.direction = DIRECTION_INVALID;  /* 标记未计算 */

    /* 调用新函数完成实际工作 */
    kalman_filter_predict_with_state(kf, &current_state, velocity, steering_angle, timestamp);
}

void kalman_filter_update(KalmanFilter_t *kf,
                         const SensorReadings_t *sensor_data) {
    if (!kf || !sensor_data) {
        return;
    }
    
    /* 根据融合模式选择算法(新增) */
    switch (kf->fusion_config.mode) {
        case FUSION_MODE_BASIC:
            kalman_filter_update_basic(kf, sensor_data);
            break;
        case FUSION_MODE_IMU_STEERING:
            kalman_filter_update_imu_steering(kf, sensor_data);
            break;
        default:
            kalman_filter_update_basic(kf, sensor_data);  /* 默认基础算法 */
            break;
    }
}

/**
 * @brief 基础融合算法：3维观测(车轮转角+轮速+IMU加速度)(新增)
 */
void kalman_filter_update_basic(KalmanFilter_t *kf,
                                const SensorReadings_t *sensor_data) {
    if (!kf || !sensor_data) {
        return;
    }
    
    /* 检测轮滑动 */
    double accel_mag = sqrt(sensor_data->imu.accel_x * sensor_data->imu.accel_x +
                           sensor_data->imu.accel_y * sensor_data->imu.accel_y);
    
    kf->diagnostics.is_wheel_slipping = (accel_mag > WHEEL_SLIP_ACCEL_THRESHOLD);
    kf->diagnostics.wheel_slip_accel = accel_mag;
    
    /* 检测静止 */
    kf->diagnostics.is_stationary = (fabs(sensor_data->wheel_odometry.velocity) < V_STATIC_THRESHOLD);
    
    /* 测量向量 [wheel_velocity, imu_accel_magnitude, steering_angle] */
    double z[3] = {
        sensor_data->wheel_odometry.velocity * kf->calibration.wheel_calibration_factor,
        accel_mag,
        sensor_data->steering.steering_angle - kf->calibration.steering_angle_offset
    };
    
    /* 测量噪声（自适应） */
    double R[3][3] = {0};
    R[0][0] = kf->noise.r_wheel * kf->noise.r_wheel;
    R[1][1] = kf->noise.r_imu * kf->noise.r_imu;
    R[2][2] = kf->noise.r_steering * kf->noise.r_steering;
    
    /* 轮滑动时增加轮速计噪声权重 */
    if (kf->diagnostics.is_wheel_slipping) {
        R[0][0] *= 4.0;
    }
    
    /* === 关键调整：当速度非常小时，增加轮速计观测的噪声权重 ===
     * 原因：在低速或停止时，脉冲计数器的离散化误差非常显著
     * 不应该让微弱的轮速测量完全压制motion_model的预测
     * 这样可以允许基于steering_angle和heading变化的预测更新
     */
    if (fabs(sensor_data->wheel_odometry.velocity) < 0.1) {
        /* 低速时，轮速计观测噪声增加10倍，降低其影响力 */
        R[0][0] *= 10.0;
    }
    
    /* 测量模型的雅可比矩阵 H (3x5) */
    double H[3][5] = {0};
    H[0][3] = 1.0;  /* 轮速计测量速度 */
    H[1][3] = 1.0;  /* IMU加速度与速度相关 */
    H[2][4] = 1.0;  /* 转向角直接测量 */
    
    /* 预测测量值 */
    double z_hat[3] = {
        kf->state.x_hat[3],  /* 预测速度 */
        0.0,                 /* IMU加速度（一阶模型为0） */
        kf->state.x_hat[4]   /* 预测转向角 */
    };
    
    /* 残差 */
    double y[3];
    y[0] = z[0] - z_hat[0];
    y[1] = z[1] - z_hat[1];
    y[2] = normalize_angle(z[2] - z_hat[2]);
    
    kf->diagnostics.residual_norm = sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    
    /* 卡尔曼增益计算：K = P * H^T * (H * P * H^T + R)^-1 */
    
    /* H * P */
    double HP[3][5] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                HP[i][j] += H[i][k] * kf->state.P[k][j];
            }
        }
    }
    
    /* H * P * H^T */
    double HPHT[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 5; k++) {
                HPHT[i][j] += HP[i][k] * H[j][k];
            }
        }
    }
    
    /* S = H * P * H^T + R */
    double S[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = HPHT[i][j] + R[i][j];
        }
    }
    
    /* S^-1 */
    double S_inv[3][3];
    if (matrix_inverse_3x3((double *)S, (double *)S_inv) != 0) {
        return;  /* 矩阵奇异，跳过更新 */
    }
    
    /* P * H^T */
    double PHT[5][3] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 5; k++) {
                PHT[i][j] += kf->state.P[i][k] * H[j][k];
            }
        }
    }
    
    /* K = P * H^T * S^-1 */
    double K[5][3] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }
    }
    
    /* 状态更新：x = x + K * y */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 3; j++) {
            kf->state.x_hat[i] += K[i][j] * y[j];
        }
    }
    
    /* 正规化角度 */
    kf->state.x_hat[2] = normalize_angle(kf->state.x_hat[2]);
    kf->state.x_hat[4] = normalize_angle(kf->state.x_hat[4]);
    
    /* 协方差更新：P = (I - K * H) * P */
    double I_KH[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double KH = 0;
            for (int k = 0; k < 3; k++) {
                KH += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH;
        }
    }
    
    double P_new[5][5] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                P_new[i][j] += I_KH[i][k] * kf->state.P[k][j];
            }
        }
    }
    
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            kf->state.P[i][j] = P_new[i][j];
        }
    }
    
    kf->diagnostics.update_count++;
}

/**
 * @brief IMU转向融合算法：4维观测(车轮转角+轮速+IMU加速度+IMU推导转向角)(新增)
 */
void kalman_filter_update_imu_steering(KalmanFilter_t *kf,
                                       const SensorReadings_t *sensor_data) {
    if (!kf || !sensor_data) {
        return;
    }
    
    double v = sensor_data->wheel_odometry.velocity;
    double accel_y = sensor_data->imu.accel_y;
    
    /* 检测轮滑动 */
    double accel_mag = sqrt(sensor_data->imu.accel_x * sensor_data->imu.accel_x +
                           sensor_data->imu.accel_y * sensor_data->imu.accel_y);
    
    kf->diagnostics.is_wheel_slipping = (accel_mag > WHEEL_SLIP_ACCEL_THRESHOLD);
    kf->diagnostics.wheel_slip_accel = accel_mag;
    
    /* 检测静止 */
    kf->diagnostics.is_stationary = (fabs(v) < V_STATIC_THRESHOLD);
    
    /* 提取或计算IMU推导转向角 */
    double delta_imu = 0.0;
    bool has_valid_imu_steering = false;
    
    if (fabs(v) > kf->fusion_config.imu_steering_vel_threshold) {
        /* IMU横向加速度校准 */
        double a_y_calibrated = (accel_y - kf->calibration.imu_lateral_accel_bias) 
                              * kf->calibration.imu_lateral_accel_scale;
        
        /* 计算IMU推导转向角：δ_imu = arctan(a_y * L / v²) */
        delta_imu = atan2(a_y_calibrated * WHEELBASE, v * v);
        has_valid_imu_steering = true;
    }
    
    /* 根据速度自适应权重调整 */
    double imu_weight = kf->fusion_config.imu_steering_weight;
    if (kf->fusion_config.enable_adaptive_weighting) {
        /* 低速时降低IMU权重，高速时提高权重 */
        if (fabs(v) < 2.0) {
            imu_weight *= 0.5;
        } else if (fabs(v) > 10.0) {
            imu_weight *= 1.5;
        }
        imu_weight = (imu_weight > 1.0) ? 1.0 : imu_weight;
    }
    
    if (!has_valid_imu_steering) {
        /* 速度过低，退回到基础算法 */
        kalman_filter_update_basic(kf, sensor_data);
        return;
    }
    
    /* 测量向量 [wheel_velocity, imu_accel_magnitude, steering_angle, imu_steering] */
    double z[4] = {
        v * kf->calibration.wheel_calibration_factor,
        accel_mag,
        sensor_data->steering.steering_angle - kf->calibration.steering_angle_offset,
        delta_imu
    };
    
    /* 测量噪声（4x4矩阵） */
    double R[4][4] = {0};
    R[0][0] = kf->noise.r_wheel * kf->noise.r_wheel;
    R[1][1] = kf->noise.r_imu * kf->noise.r_imu;
    R[2][2] = kf->noise.r_steering * kf->noise.r_steering;
    R[3][3] = (kf->noise.r_imu_steering > 0) 
             ? (kf->noise.r_imu_steering * kf->noise.r_imu_steering)
             : (0.1 * 0.1);  /* 默认值 */
    
    /* 轮滑动时增加轮速计噪声权重 */
    if (kf->diagnostics.is_wheel_slipping) {
        R[0][0] *= 4.0;
    }
    
    /* 降低IMU权重 */
    R[3][3] /= (imu_weight * imu_weight + 0.01);  /* 避免除以零 */
    
    /* 测量模型的雅可比矩阵 H (4x5) */
    double H[4][5] = {0};
    H[0][3] = 1.0;  /* 轮速计测量速度 */
    H[1][3] = 1.0;  /* IMU加速度与速度相关 */
    H[2][4] = 1.0;  /* 转向角直接测量 */
    H[3][4] = 1.0;  /* IMU推导转向角观测 */
    
    /* 预测测量值 */
    double z_hat[4] = {
        kf->state.x_hat[3],  /* 预测速度 */
        0.0,                 /* IMU加速度（一阶模型为0） */
        kf->state.x_hat[4],  /* 预测转向角 */
        kf->state.x_hat[4]   /* 预测转向角同时作为IMU推导转向角预测 */
    };
    
    /* 残差 */
    double y[4];
    y[0] = z[0] - z_hat[0];
    y[1] = z[1] - z_hat[1];
    y[2] = normalize_angle(z[2] - z_hat[2]);
    y[3] = normalize_angle(z[3] - z_hat[3]);
    
    kf->diagnostics.residual_norm = sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2] + y[3]*y[3]);
    
    /* 卡尔曼增益计算：K = P * H^T * (H * P * H^T + R)^-1 */
    
    /* H * P */
    double HP[4][5] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                HP[i][j] += H[i][k] * kf->state.P[k][j];
            }
        }
    }
    
    /* H * P * H^T */
    double HPHT[4][4] = {0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 5; k++) {
                HPHT[i][j] += HP[i][k] * H[j][k];
            }
        }
    }
    
    /* S = H * P * H^T + R */
    double S[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            S[i][j] = HPHT[i][j] + R[i][j];
        }
    }
    
    /* S^-1 (使用4x4矩阵逆) */
    double S_inv[4][4];
    if (matrix_inverse_4x4((double (*)[4])S, (double (*)[4])S_inv) != 0) {
        /* 如果4x4矩阵求逆失败，退回到基础算法 */
        kalman_filter_update_basic(kf, sensor_data);
        return;
    }
    
    /* P * H^T */
    double PHT[5][4] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 5; k++) {
                PHT[i][j] += kf->state.P[i][k] * H[j][k];
            }
        }
    }
    
    /* K = P * H^T * S^-1 */
    double K[5][4] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 4; k++) {
                K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }
    }
    
    /* 状态更新：x = x + K * y */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 4; j++) {
            kf->state.x_hat[i] += K[i][j] * y[j];
        }
    }
    
    /* 正规化角度 */
    kf->state.x_hat[2] = normalize_angle(kf->state.x_hat[2]);
    kf->state.x_hat[4] = normalize_angle(kf->state.x_hat[4]);
    
    /* 协方差更新：P = (I - K * H) * P */
    double I_KH[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            double KH = 0;
            for (int k = 0; k < 4; k++) {
                KH += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH;
        }
    }
    
    double P_new[5][5] = {0};
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            for (int k = 0; k < 5; k++) {
                P_new[i][j] += I_KH[i][k] * kf->state.P[k][j];
            }
        }
    }
    
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            kf->state.P[i][j] = P_new[i][j];
        }
    }
    
    kf->diagnostics.update_count++;
}

void kalman_filter_get_state(const KalmanFilter_t *kf,
                            VehicleState_t *state) {
    if (!kf || !state) {
        return;
    }
    
    /* 将内部 double 精度状态转换为外部 float 版本 */
    state->x = (float)kf->state.x_hat[0];
    state->y = (float)kf->state.x_hat[1];
    state->heading = (float)kf->state.x_hat[2];
    state->v = (float)kf->state.x_hat[3];
    state->steering_angle = (float)kf->state.x_hat[4];
    state->timestamp = kf->state.previous_timestamp;
    state->gear = kf->state.previous_gear;
    state->direction = DIRECTION_INVALID;
}

DiagnosticsInfo_t kalman_filter_get_diagnostics(const KalmanFilter_t *kf) {
    if (!kf) {
        DiagnosticsInfo_t empty = {0};
        return empty;
    }
    return kf->diagnostics;
}

void kalman_filter_reset(KalmanFilter_t *kf,
                        const VehicleState_t *initial_state) {
    if (!kf || !initial_state) {
        return;
    }
    
    /* 将 float 外部 API 转换为内部 double 精度 */
    VehicleState_internal_t state_internal;
    vehicle_state_float_to_internal(initial_state, &state_internal);
    
    kf->state.x_hat[0] = state_internal.x;
    kf->state.x_hat[1] = state_internal.y;
    kf->state.x_hat[2] = state_internal.theta;
    kf->state.x_hat[3] = state_internal.v;
    kf->state.x_hat[4] = state_internal.delta;
    
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            kf->state.P[i][j] = (i == j) ? 0.1 : 0.0;
        }
    }
    
    kf->state.previous_timestamp = 0;
    memset(&kf->diagnostics, 0, sizeof(kf->diagnostics));
}

/* ========== 转向角抖动过滤器实现 ========== */

/**
 * @brief 计算转向角统计量（均值和方差）
 */
static void compute_steering_stats(SteeringJitterFilter_t *filter) {
    if (filter->history_count == 0) {
        filter->mean_angle = 0.0;
        filter->angle_variance = 0.0;
        return;
    }
    
    /* 计算均值 */
    double sum = 0.0;
    for (uint32_t i = 0; i < filter->history_count; i++) {
        sum += filter->angle_history[i];
    }
    filter->mean_angle = sum / filter->history_count;
    
    /* 计算方差 */
    double sum_sq = 0.0;
    for (uint32_t i = 0; i < filter->history_count; i++) {
        double diff = filter->angle_history[i] - filter->mean_angle;
        sum_sq += diff * diff;
    }
    filter->angle_variance = sum_sq / filter->history_count;
}

int steering_jitter_filter_init(SteeringJitterFilter_t *filter) {
    if (!filter) {
        return -1;
    }
    
    memset(filter, 0, sizeof(SteeringJitterFilter_t));
    
    /* 初始化参数 */
    filter->jitter_threshold = 0.03;    /* 30 mrad = 1.7° */
    filter->filter_alpha = 0.2;         /* 低通滤波系数 */
    filter->history_count = 0;
    filter->history_index = 0;
    filter->mean_angle = 0.0;
    filter->angle_variance = 0.0;
    filter->zero_bias_angle = 0.0;
    filter->filtered_angle = 0.0;
    filter->update_count = 0;
    filter->is_high_jitter = false;
    
    return 0;
}

int steering_jitter_filter_update(SteeringJitterFilter_t *filter,const SteeringConfig_t *steering_config,
                                   const double steering_angle) {
    if (!filter) {
        return -1;
    }
    
    /* 限制转向角范围 */
    double clamped_angle = clamp(steering_angle, -steering_config->max_steering_wheel_angle, steering_config->max_steering_wheel_angle);
    
    /* 更新低通过滤 */
    if (filter->update_count == 0) {
        filter->filtered_angle = clamped_angle;
    } else {
        filter->filtered_angle = filter->filter_alpha * clamped_angle + 
                                (1.0 - filter->filter_alpha) * filter->filtered_angle;
    }
    
    /* 将角度添加到历史缓冲 */
    filter->angle_history[filter->history_index] = clamped_angle;
    filter->history_index = (filter->history_index + 1) % 10;
    
    if (filter->history_count < 10) {
        filter->history_count++;
    }
    
    /* 计算统计量 */
    compute_steering_stats(filter);
    
    /* 检测高抖动 */
    filter->is_high_jitter = (filter->angle_variance > filter->jitter_threshold);
    
    filter->update_count++;
    
    return 0;
}

int steering_jitter_filter_compute_offset(SteeringJitterFilter_t *filter,
                                          double velocity) {
    if (!filter) {
        return -1;
    }
    
    /* 仅在低速或静止时计算零偏，以提高准确性 */
    if (fabs(velocity) > 0.2) {
        return -1;  /* 速度过高，条件不满足 */
    }
    
    /* 需要足够的历史数据 */
    if (filter->history_count < 5) {
        return -1;  /* 数据不足 */
    }
    
    /* 检查方差是否在可接受范围内 */
    if (filter->angle_variance > 0.05) {
        return -1;  /* 抖动过大，数据不稳定 */
    }
    
    /* 更新零偏角为当前均值 */
    filter->zero_bias_angle = filter->mean_angle;
    
    return 0;
}

int steering_jitter_filter_get_stats(const SteeringJitterFilter_t *filter,
                                     double *mean_angle,
                                     double *variance,
                                     double *zero_bias,
                                     bool *is_high_jitter) {
    if (!filter) {
        return -1;
    }
    
    if (mean_angle) *mean_angle = filter->mean_angle;
    if (variance) *variance = filter->angle_variance;
    if (zero_bias) *zero_bias = filter->zero_bias_angle;
    if (is_high_jitter) *is_high_jitter = filter->is_high_jitter;
    
    return 0;
}

int steering_jitter_filter_set_params(SteeringJitterFilter_t *filter,
                                       double jitter_threshold,
                                       double filter_alpha) {
    if (!filter) {
        return -1;
    }
    
    /* 参数校验 */
    if (jitter_threshold < 0.001 || jitter_threshold > 0.1) {
        return -1;  /* 抖动阈值范围：0.001-0.1 rad */
    }
    
    if (filter_alpha < 0.0 || filter_alpha > 1.0) {
        return -1;  /* 滤波系数应在 [0, 1] 范围内 */
    }
    
    filter->jitter_threshold = jitter_threshold;
    filter->filter_alpha = filter_alpha;
    
    return 0;
}

void steering_jitter_filter_reset(SteeringJitterFilter_t *filter) {
    if (!filter) {
        return;
    }
    
    memset(filter->angle_history, 0, sizeof(filter->angle_history));
    filter->history_index = 0;
    filter->history_count = 0;
    filter->mean_angle = 0.0;
    filter->angle_variance = 0.0;
    filter->filtered_angle = 0.0;
    filter->update_count = 0;
    filter->is_high_jitter = false;
    /* 保留零偏角和参数 */
}

/* ========== Float API Implementation (for TC397 MCU compatibility) ========== */

/**
 * @brief 预测步骤 (Float版本)
 * 
 * 内部将float参数转换为double后调用原有的double版本函数
 */
void kalman_filter_predict_float(KalmanFilter_t *kf,
                                 float velocity,
                                 float steering_angle,
                                 uint64_t timestamp) {
    if (!kf) {
        return;
    }
    
    /* 转换float为double并调用原有函数 */
    kalman_filter_predict(kf,
                         (double)velocity,
                         (double)steering_angle,
                         timestamp);
}

/**
 * @brief 更新步骤 (Float版本)
 * 
 * 直接转发到原有的double版本函数
 * (SensorReadings_t中的数据已是double精度)
 */
void kalman_filter_update_float(KalmanFilter_t *kf,
                                const SensorReadings_t *sensor_data) {
    if (!kf || !sensor_data) {
        return;
    }
    
    /* 直接调用double版本 */
    kalman_filter_update(kf, sensor_data);
}

/**
 * @brief 获取当前状态估计 (Float版本)
 * 
 * 直接转发到原有的double版本函数
 * 返回的VehicleState_t保持double精度
 */
void kalman_filter_get_state_float(const KalmanFilter_t *kf,
                                   VehicleState_t *state) {
    if (!kf || !state) {
        return;
    }
    
    /* 直接调用double版本 */
    kalman_filter_get_state(kf, state);
}

/**
 * @brief 设置车辆当前档位
 * 用于更新方向判断的档位信息
 */
void kalman_filter_set_gear(KalmanFilter_t *kf, VehicleGear_t gear) {
    if (kf && gear != GEAR_INVALID) {
        kf->state.previous_gear = gear;
    }
}
