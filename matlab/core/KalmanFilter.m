% KalmanFilter.m - 扩展卡尔曼滤波器 (EKF) 类
% 实现两种融合算法的完整EKF系统

classdef KalmanFilter < handle
    properties
        % 滤波状态
        x_hat = zeros(5, 1)                % 状态估计向量 [x, y, theta, v, delta]'
        P = eye(5)                         % 协方差矩阵 5x5
        
        % 配置参数
        noise = NoiseParams()              % 噪声参数
        calibration = CalibrationParams()  % 校准参数
        fusion_config = FusionConfig()     % 融合配置
        
        % 算法模块
        motion_model = MotionModel()       % 运动模型
        steering_model = SteeringModel()   % 转向模型
        
        % 诊断信息
        prev_timestamp = 0                 % 前一时刻时间戳 (s)
        innovation_history = []            % 创新序列历史
    end
    
    methods
        % 构造函数
        function obj = KalmanFilter()
            % KalmanFilter 构造函数
            obj.x_hat = [0; 0; 0; 0; 0];
            obj.P = eye(5);
            obj.noise = NoiseParams();
            obj.calibration = CalibrationParams();
            obj.fusion_config = FusionConfig();
            obj.motion_model = MotionModel();
            obj.steering_model = SteeringModel();
            obj.prev_timestamp = 0;
            obj.innovation_history = [];
        end
        
        % 初始化滤波器
        function init(obj, initial_state, noise, calibration, fusion_config)
            % 初始化卡尔曼滤波器
            % 参数:
            %   initial_state - VehicleState 初始状态
            %   noise - NoiseParams 噪声参数
            %   calibration - CalibrationParams 校准参数
            %   fusion_config - FusionConfig 融合配置
            
            % 从初始状态设置状态估计
            obj.x_hat = initial_state.toVector();
            
            % 初始化协方差矩阵
            obj.P = eye(5) * 0.1;
            
            % 设置配置参数
            obj.noise = noise;
            obj.calibration = calibration;
            obj.fusion_config = fusion_config;
            
            % 重置时间戳
            obj.prev_timestamp = initial_state.timestamp;
        end
        
        % 预测步骤
        function predict(obj, velocity, steering_angle, timestamp)
            % EKF预测步骤
            % 参数:
            %   velocity - 纵向速度 (m/s)
            %   steering_angle - 转向角 (rad)
            %   timestamp - 当前时间戳 (s)
            
            % 计算时间间隔
            if obj.prev_timestamp > 0
                dt = timestamp - obj.prev_timestamp;
            else
                dt = 0.05;  % 默认50ms
            end
            
            % 应用校准参数
            v_calibrated = obj.calibration.calibrate_wheel_velocity(velocity);
            delta_calibrated = obj.calibration.calibrate_steering_angle(steering_angle);
            
            % 更新转向模型状态
            steering_angle_filtered = obj.steering_model.update(delta_calibrated, timestamp);
            
            % 从当前状态估计构建VehicleState对象
            state = VehicleState();
            state = state.fromVector(obj.x_hat);
            
            % 运动模型预测
            new_state = obj.motion_model.predict(state, v_calibrated, steering_angle_filtered, dt);
            
            % 更新状态估计
            obj.x_hat = new_state.toVector();
            
            % 计算雅可比矩阵
            F = obj.motion_model.jacobian(state, v_calibrated, steering_angle_filtered, dt);
            
            % 协方差预测: P = F*P*F' + Q
            Q = obj.noise.getQMatrix();
            obj.P = F * obj.P * F' + Q;
            
            % 保存时间戳
            obj.prev_timestamp = timestamp;
        end
        
        % 更新步骤 (多分派路由)
        function update(obj, sensor_data)
            % EKF更新步骤
            % 根据融合模式自动选择相应的更新算法
            % 参数:
            %   sensor_data - SensorData 传感器数据
            
            if obj.fusion_config.isIMUSteering()
                obj.update_imu_steering(sensor_data);
            else
                obj.update_basic(sensor_data);
            end
        end
        
        % 基础融合算法 (3维观测)
        function update_basic(obj, sensor_data)
            % 基础融合: 轮速 + 转向角
            % 参数:
            %   sensor_data - SensorData 传感器数据
            
            % 观测向量: z = [v_wheel, delta_wheel]'
            z = [
                sensor_data.wheel_velocity;
                sensor_data.steering_angle
            ];
            
            % 观测模型Jacobian矩阵 H (2x5)
            H = zeros(2, 5);
            H(1, 4) = 1;  % 速度在状态向量的第4个位置
            H(2, 5) = 1;  % 转向角在状态向量的第5个位置
            
            % 预测的观测值
            z_pred = obj.x_hat([4, 5]);
            
            % 创新 (观测残差)
            innovation = z - z_pred;
            innovation(2) = angle_utils.normalize_angle(innovation(2));
            
            % 测量噪声协方差 R (2x2)
            R = obj.noise.getRMatrixBasic();
            
            % 卡尔曼增益: K = P*H'*inv(H*P*H' + R)
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            % 状态更新
            obj.x_hat = obj.x_hat + K * innovation;
            
            % 规范化航向角
            obj.x_hat(3) = angle_utils.normalize_angle(obj.x_hat(3));
            
            % 协方差更新: P = (I - K*H)*P
            I = eye(5);
            obj.P = (I - K * H) * obj.P;
            
            % 记录创新
            obj.innovation_history = [obj.innovation_history; norm(innovation)];
            if length(obj.innovation_history) > 100
                obj.innovation_history = obj.innovation_history(end-99:end);
            end
        end
        
        % IMU+转向融合算法 (4维观测)
        function update_imu_steering(obj, sensor_data)
            % 增强融合: 轮速 + IMU加速度 + 转向角 + IMU推导转向角
            % 参数:
            %   sensor_data - SensorData 传感器数据
            
            % 检查速度
            velocity = obj.x_hat(4);
            if abs(velocity) < constants.V_STATIC
                % 低速时退回到基础融合
                obj.update_basic(sensor_data);
                return;
            end
            
            % 应用校准
            accel_calibrated = obj.calibration.calibrate_imu_accel( ...
                sensor_data.imu_accel_x, ...
                sensor_data.imu_accel_y, ...
                sensor_data.imu_accel_z);
            accel_mag = norm(accel_calibrated);
            
            % 观测向量: z = [v_wheel, a_mag, delta_wheel]'
            z = [
                sensor_data.wheel_velocity;
                accel_mag;
                sensor_data.steering_angle
            ];
            
            % 观测模型Jacobian矩阵 H (3x5)
            H = zeros(3, 5);
            H(1, 4) = 1;  % 速度
            H(2, 4) = 0;  % 加速度（近似为常数）
            H(3, 5) = 1;  % 转向角
            
            % 预测的观测值
            z_pred = [obj.x_hat(4); 0; obj.x_hat(5)];
            
            % 创新 (观测残差)
            innovation = z - z_pred;
            innovation(3) = angle_utils.normalize_angle(innovation(3));
            
            % 测量噪声协方差 R (3x3)
            R = obj.noise.getRMatrixIMUSteering();
            
            % 计算自适应权重
            weight = obj.fusion_config.getAdaptiveWeight(velocity);
            R(2, 2) = R(2, 2) / (weight + 0.1);  % 低权重 = 高噪声
            
            % 卡尔曼增益: K = P*H'*inv(H*P*H' + R)
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            % 状态更新
            obj.x_hat = obj.x_hat + K * innovation;
            
            % 规范化航向角
            obj.x_hat(3) = angle_utils.normalize_angle(obj.x_hat(3));
            
            % 协方差更新: P = (I - K*H)*P
            I = eye(5);
            obj.P = (I - K * H) * obj.P;
            
            % 记录创新
            obj.innovation_history = [obj.innovation_history; norm(innovation)];
            if length(obj.innovation_history) > 100
                obj.innovation_history = obj.innovation_history(end-99:end);
            end
        end
        
        % 获取当前状态估计
        function state = get_state(obj)
            % 返回当前状态估计
            % 返回:
            %   state - VehicleState 对象
            
            state = VehicleState();
            state = state.fromVector(obj.x_hat);
            state.timestamp = obj.prev_timestamp;
        end
        
        % 获取协方差矩阵
        function P = get_covariance(obj)
            % 返回当前协方差矩阵
            P = obj.P;
        end
        
        % 获取位置估计标准差
        function sigma = get_position_sigma(obj)
            % 返回位置估计的标准差 [sigma_x, sigma_y]
            sigma = [sqrt(obj.P(1, 1)); sqrt(obj.P(2, 2))];
        end
        
        % 获取航向角估计标准差
        function sigma = get_heading_sigma(obj)
            % 返回航向角估计的标准差 (rad)
            sigma = sqrt(obj.P(3, 3));
        end
        
        % 显示滤波器状态
        function display(obj)
            fprintf('KalmanFilter State:\n');
            fprintf('  Estimated State:\n');
            fprintf('    Position: (%.3f, %.3f) m\n', obj.x_hat(1), obj.x_hat(2));
            fprintf('    Heading: %.3f rad (%.1f deg)\n', obj.x_hat(3), obj.x_hat(3) * constants.RAD_TO_DEG);
            fprintf('    Velocity: %.3f m/s\n', obj.x_hat(4));
            fprintf('    Steering Angle: %.3f rad (%.1f deg)\n', obj.x_hat(5), obj.x_hat(5) * constants.RAD_TO_DEG);
            fprintf('  Covariance Diagonals: [%.6f, %.6f, %.6f, %.6f, %.6f]\n', ...
                    diag(obj.P)');
            fprintf('  Fusion Mode: %s\n', constants.getFusionModeName(obj.fusion_config.mode));
        end
    end
end
