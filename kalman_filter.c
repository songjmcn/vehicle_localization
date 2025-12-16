/**
 * @file kalman_filter.c
 * @brief 扩展卡尔曼滤波器实现
 */

#include "kalman_filter.h"
#include "motion_model.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* ========== 矩阵操作函数 ========== */

/**
 * @brief 矩阵乘法 C = A * B (n x m) * (m x p)
 */
static void matrix_multiply(double *A, double *B, double *C,
                           int n, int m, int p) {
    memset(C, 0, n * p * sizeof(double));
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < p; j++) {
            for (int k = 0; k < m; k++) {
                C[i * p + j] += A[i * m + k] * B[k * p + j];
            }
        }
    }
}

/**
 * @brief 矩阵转置 B = A^T (n x m -> m x n)
 */
static void matrix_transpose(double *A, double *B, int n, int m) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            B[j * n + i] = A[i * m + j];
        }
    }
}

/**
 * @brief 矩阵加法 C = A + B (n x m)
 */
static void matrix_add(double *A, double *B, double *C, int n, int m) {
    for (int i = 0; i < n * m; i++) {
        C[i] = A[i] + B[i];
    }
}

/**
 * @brief 矩阵减法 C = A - B (n x m)
 */
static void matrix_subtract(double *A, double *B, double *C, int n, int m) {
    for (int i = 0; i < n * m; i++) {
        C[i] = A[i] - B[i];
    }
}

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
static int matrix_inverse_5x5(double A[5][5], double A_inv[5][5]) {
    double temp[5][10];
    
    /* 构造增广矩阵 [A | I] */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            temp[i][j] = A[i][j];
            temp[i][j + 5] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    /* 高斯消元 */
    for (int i = 0; i < 5; i++) {
        /* 找主元 */
        int pivot = i;
        for (int k = i + 1; k < 5; k++) {
            if (fabs(temp[k][i]) > fabs(temp[pivot][i])) {
                pivot = k;
            }
        }
        
        if (fabs(temp[pivot][i]) < 1e-10) {
            return -1;  /* 矩阵奇异 */
        }
        
        /* 行交换 */
        if (pivot != i) {
            for (int j = 0; j < 10; j++) {
                double t = temp[i][j];
                temp[i][j] = temp[pivot][j];
                temp[pivot][j] = t;
            }
        }
        
        /* 主元化为1 */
        double div = temp[i][i];
        for (int j = 0; j < 10; j++) {
            temp[i][j] /= div;
        }
        
        /* 消元 */
        for (int k = 0; k < 5; k++) {
            if (k != i) {
                double mul = temp[k][i];
                for (int j = 0; j < 10; j++) {
                    temp[k][j] -= mul * temp[i][j];
                }
            }
        }
    }
    
    /* 提取逆矩阵 */
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            A_inv[i][j] = temp[i][j + 5];
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
    
    /* 初始化状态 */
    kf->state.x_hat[0] = initial_state->x;
    kf->state.x_hat[1] = initial_state->y;
    kf->state.x_hat[2] = initial_state->theta;
    kf->state.x_hat[3] = initial_state->v;
    kf->state.x_hat[4] = initial_state->delta;
    
    /* 初始化协方差矩阵 */
    memset(kf->state.P, 0, sizeof(kf->state.P));
    for (int i = 0; i < 5; i++) {
        kf->state.P[i][i] = 0.1;
    }
    
    /* 保存噪声参数和校准参数 */
    kf->noise = *noise;
    kf->calibration = *calibration;
    
    /* 初始化时间戳 */
    kf->state.previous_timestamp = 0;
    
    /* 初始化诊断信息 */
    memset(&kf->diagnostics, 0, sizeof(kf->diagnostics));
    
    return 0;
}

void kalman_filter_predict(KalmanFilter_t *kf,
                          double velocity,
                          double steering_angle,
                          uint32_t timestamp) {
    if (!kf || kf->state.previous_timestamp == 0) {
        kf->state.previous_timestamp = timestamp;
        return;
    }
    
    double dt = (timestamp - kf->state.previous_timestamp) / 1000.0;
    if (dt <= 0 || dt > 1.0) {
        kf->state.previous_timestamp = timestamp;
        return;
    }
    
    /* 应用校准参数 */
    double cal_velocity = velocity * kf->calibration.wheel_calibration_factor;
    double cal_steering = steering_angle - kf->calibration.steering_angle_offset;
    cal_steering *= kf->calibration.steering_angle_gain;
    cal_steering = clamp(cal_steering, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX);
    
    /* 当前状态 */
    VehicleState_t current_state;
    current_state.x = kf->state.x_hat[0];
    current_state.y = kf->state.x_hat[1];
    current_state.theta = kf->state.x_hat[2];
    current_state.v = kf->state.x_hat[3];
    current_state.delta = kf->state.x_hat[4];
    
    /* 运动模型预测 */
    VehicleState_t predicted_state = motion_model_predict(&current_state,
                                                           cal_velocity,
                                                           cal_steering,
                                                           dt);
    
    /* 更新状态 */
    kf->state.x_hat[0] = predicted_state.x;
    kf->state.x_hat[1] = predicted_state.y;
    kf->state.x_hat[2] = predicted_state.theta;
    kf->state.x_hat[3] = predicted_state.v;
    kf->state.x_hat[4] = predicted_state.delta;
    
    /* 计算雅可比矩阵 */
    double F[5][5];
    motion_model_jacobian(&current_state, cal_velocity, cal_steering, dt, F);
    
    /* 协方差预测：P = F * P * F^T + Q */
    double F_P[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            F_P[i][j] = 0;
            for (int k = 0; k < 5; k++) {
                F_P[i][j] += F[i][k] * kf->state.P[k][j];
            }
        }
    }
    
    double F_T[5][5];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            F_T[i][j] = F[j][i];
        }
    }
    
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            kf->state.P[i][j] = 0;
            for (int k = 0; k < 5; k++) {
                kf->state.P[i][j] += F_P[i][k] * F_T[k][j];
            }
        }
    }
    
    /* 加入过程噪声 */
    kf->state.P[0][0] += kf->noise.q_x * kf->noise.q_x;
    kf->state.P[1][1] += kf->noise.q_y * kf->noise.q_y;
    kf->state.P[2][2] += kf->noise.q_theta * kf->noise.q_theta;
    kf->state.P[3][3] += kf->noise.q_v * kf->noise.q_v;
    kf->state.P[4][4] += kf->noise.q_delta * kf->noise.q_delta;
    
    kf->state.previous_timestamp = timestamp;
}

void kalman_filter_update(KalmanFilter_t *kf,
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

void kalman_filter_get_state(const KalmanFilter_t *kf,
                            VehicleState_t *state) {
    if (!kf || !state) {
        return;
    }
    
    state->x = kf->state.x_hat[0];
    state->y = kf->state.x_hat[1];
    state->theta = kf->state.x_hat[2];
    state->v = kf->state.x_hat[3];
    state->delta = kf->state.x_hat[4];
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
    
    kf->state.x_hat[0] = initial_state->x;
    kf->state.x_hat[1] = initial_state->y;
    kf->state.x_hat[2] = initial_state->theta;
    kf->state.x_hat[3] = initial_state->v;
    kf->state.x_hat[4] = initial_state->delta;
    
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

int steering_jitter_filter_update(SteeringJitterFilter_t *filter,
                                   double steering_angle) {
    if (!filter) {
        return -1;
    }
    
    /* 限制转向角范围 */
    double clamped_angle = clamp(steering_angle, -STEERING_ANGLE_MAX, STEERING_ANGLE_MAX);
    
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
