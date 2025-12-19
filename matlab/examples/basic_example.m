% basic_example.m - 基础使用示例
% 演示如何使用MATLAB定位系统

% 清空环境
clear all; close all; clc;

% ========== 第1步：路径配置 ==========
fprintf('===== 车辆定位系统 MATLAB 实现 - 基础示例 =====\n\n');

% 添加路径
matlab_root = fileparts(mfilename('fullpath'));
matlab_root = fileparts(matlab_root);  % 返回到matlab文件夹

addpath(fullfile(matlab_root, 'data_structures'));
addpath(fullfile(matlab_root, 'utils'));
addpath(fullfile(matlab_root, 'core'));
addpath(matlab_root);

fprintf('✓ 已加载MATLAB模块\n\n');

% ========== 第2步：初始化参数 ==========
fprintf('===== 初始化系统参数 =====\n');

% 创建初始状态
initial_state = VehicleState(0, 0, 0, 0, 0, 0, constants.GEAR_NEUTRAL, constants.DIRECTION_STATIONARY);
fprintf('✓ 初始位置: (%.1f, %.1f) m\n', initial_state.x, initial_state.y);

% 创建噪声参数
noise = NoiseParams( ...
    'q_x', 0.01, ...
    'q_y', 0.01, ...
    'q_theta', 0.001, ...
    'q_v', 0.05, ...
    'q_delta', 0.01, ...
    'r_wheel', 0.05, ...
    'r_imu', 0.3, ...
    'r_steering', 0.01 ...
);
fprintf('✓ 配置噪声参数\n');

% 创建校准参数
calibration = CalibrationParams( ...
    'wheel_calibration_factor', 1.0, ...
    'steering_angle_offset', 0.0, ...
    'steering_angle_gain', 1.0 ...
);
fprintf('✓ 配置校准参数\n');

% 创建融合配置 (基础模式)
fusion_config = FusionConfig('mode', constants.FUSION_MODE_BASIC);
fprintf('✓ 融合模式: %s\n\n', constants.getFusionModeName(fusion_config.mode));

% ========== 第3步：初始化滤波器 ==========
fprintf('===== 初始化卡尔曼滤波器 =====\n');

kf = KalmanFilter();
kf.init(initial_state, noise, calibration, fusion_config);
fprintf('✓ 卡尔曼滤波器已初始化\n\n');

% ========== 第4步：主循环 - 模拟直线行驶 ==========
fprintf('===== 开始仿真：直线行驶场景 =====\n');

% 仿真参数
dt = 0.05;  % 50ms
num_steps = 200;
target_velocity = 2.0;  % m/s (2.0 m/s)
max_steering_angle = 0.1;  % 10° = 0.1745 rad

% 记录结果
trajectory = zeros(num_steps, 5);  % [x, y, theta, v, delta]
covariance_trace = zeros(num_steps, 1);
heading_sigma = zeros(num_steps, 1);

fprintf('仿真参数: dt=%.3f s, 目标速度=%.2f m/s\n', dt, target_velocity);
fprintf('总仿真时长: %.2f s\n\n', num_steps * dt);

% 进度条
fprintf('进度: ');

for step = 1:num_steps
    % 当前时间戳
    timestamp = step * dt;
    
    % 生成模拟传感器数据
    % 前50步加速到目标速度
    if step <= 50
        velocity = target_velocity * step / 50;
    else
        velocity = target_velocity;
    end
    
    % 模拟J形轨迹：前100步直线，后面转向
    if step <= 100
        steering_angle = 0;  % 直线
    else
        % 缓慢转向
        steering_angle = max_steering_angle * sin((step - 100) / 50 * pi);
    end
    
    % 创建传感器数据（含高斯噪声）
    noise_wheel = randn() * 0.01;
    noise_steering = randn() * 0.005;
    
    sensor_data = SensorData( ...
        velocity + noise_wheel, ...      % 轮速 + 噪声
        randn() * 0.01, ...              % IMU加速度X
        randn() * 0.01, ...              % IMU加速度Y
        -9.81 + randn() * 0.01, ...      % IMU加速度Z
        randn() * 0.001, ...             % IMU陀螺仪
        steering_angle + noise_steering, ...% 转向角 + 噪声
        timestamp ...
    );
    
    % ===== EKF 预测-更新循环 =====
    % 预测
    kf.predict(sensor_data.wheel_velocity, steering_angle, timestamp);
    
    % 更新
    kf.update(sensor_data);
    
    % 获取当前估计
    state_est = kf.get_state();
    
    % 记录轨迹
    trajectory(step, :) = [ ...
        state_est.x, state_est.y, state_est.theta, ...
        state_est.v, state_est.delta ...
    ];
    
    % 记录协方差信息
    P = kf.get_covariance();
    covariance_trace(step) = trace(P);
    heading_sigma(step) = kf.get_heading_sigma();
    
    % 进度显示
    if mod(step, 20) == 0
        fprintf('.');
    end
end

fprintf('\n✓ 仿真完成\n\n');

% ========== 第5步：结果显示 ==========
fprintf('===== 仿真结果 =====\n');

final_state = trajectory(end, :);
fprintf('最终位置: (%.3f, %.3f) m\n', final_state(1), final_state(2));
fprintf('最终速度: %.3f m/s\n', final_state(4));
fprintf('最终航向: %.3f rad (%.1f deg)\n', ...
        final_state(3), final_state(3) * constants.RAD_TO_DEG);
fprintf('最终转向角: %.3f rad (%.1f deg)\n', ...
        final_state(5), final_state(5) * constants.RAD_TO_DEG);

fprintf('\n协方差信息:\n');
fprintf('平均位置标准差: (%.4f, %.4f) m\n', ...
        mean(sqrt(trajectory(:, 1).^2)), ...
        mean(sqrt(trajectory(:, 2).^2)));
fprintf('平均航向标准差: %.4f rad\n', mean(heading_sigma));
fprintf('平均协方差迹: %.6f\n\n', mean(covariance_trace));

% ========== 第6步：可视化 ==========
fprintf('===== 生成可视化图表 =====\n');

figure('Position', [100, 100, 1200, 800]);

% 子图1：轨迹
subplot(2, 3, 1);
plot(trajectory(:, 1), trajectory(:, 2), 'b-', 'LineWidth', 2);
hold on;
plot(trajectory(1, 1), trajectory(1, 2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(trajectory(end, 1), trajectory(end, 2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('车辆轨迹');
legend('估计轨迹', '起点', '终点');
axis equal;

% 子图2：速度
subplot(2, 3, 2);
time_vec = (1:num_steps) * dt;
plot(time_vec, trajectory(:, 4), 'b-', 'LineWidth', 2);
hold on;
plot(time_vec(1:100), ones(100, 1) * target_velocity, 'r--', 'LineWidth', 1);
grid on;
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('纵向速度估计');
legend('估计速度', '预期速度');

% 子图3：航向角
subplot(2, 3, 3);
heading_deg = trajectory(:, 3) * constants.RAD_TO_DEG;
plot(time_vec, heading_deg, 'b-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)');
ylabel('航向角 (deg)');
title('航向角估计');

% 子图4：转向角
subplot(2, 3, 4);
steering_deg = trajectory(:, 5) * constants.RAD_TO_DEG;
plot(time_vec, steering_deg, 'g-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)');
ylabel('转向角 (deg)');
title('转向角估计');

% 子图5：协方差迹
subplot(2, 3, 5);
semilogy(time_vec, covariance_trace, 'r-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)');
ylabel('协方差迹');
title('估计不确定性');

% 子图6：航向角标准差
subplot(2, 3, 6);
plot(time_vec, heading_sigma * constants.RAD_TO_DEG, 'm-', 'LineWidth', 2);
grid on;
xlabel('时间 (s)');
ylabel('航向标准差 (deg)');
title('航向角估计不确定性');

sgtitle('车辆定位系统仿真结果');

fprintf('✓ 可视化完成\n\n');

% ========== 结束 ==========
fprintf('===== 示例完成 =====\n');
fprintf('✓ 所有步骤成功执行\n');
fprintf('✓ 结果已显示在图表中\n');
