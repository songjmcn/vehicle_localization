% SteeringModel.m - 转向角转换和处理模块
% 实现方向盘转角到前轮转向角的转换，以及低通滤波

classdef SteeringModel
    properties
        % 配置参数
        steering_ratio = 16.0                       % 齿比
        max_steering_wheel_angle = pi               % 最大方向盘转角 (rad)
        max_front_wheel_angle = 0.628               % 最大前轮转向角 (rad)
        wheelbase = constants.WHEELBASE             % 轴距 (m)
        steering_angle_offset = 0.0                 % 零偏补偿 (rad)
        steering_angle_gain = 1.0                   % 增益补偿
        enable_steering_filter = true               % 启用低通滤波
        steering_filter_alpha = 0.15                % 滤波系数
        
        % 状态变量
        filtered_steering_angle = 0.0               % 滤波后的转向角 (rad)
        prev_steering_angle = 0.0                   % 前一时刻的转向角 (rad)
        steering_rate = 0.0                         % 转向角速率 (rad/s)
        prev_timestamp = 0                          % 前一时刻时间戳 (ms)
    end
    
    methods
        % 构造函数
        function obj = SteeringModel(varargin)
            % SteeringModel 构造函数
            % 可选参数: 'steering_ratio', 'wheelbase', 'steering_angle_offset' 等
            
            if nargin > 0
                p = inputParser;
                addParameter(p, 'steering_ratio', 16.0);
                addParameter(p, 'max_steering_wheel_angle', pi);
                addParameter(p, 'max_front_wheel_angle', 0.628);
                addParameter(p, 'wheelbase', constants.WHEELBASE);
                addParameter(p, 'steering_angle_offset', 0.0);
                addParameter(p, 'steering_angle_gain', 1.0);
                addParameter(p, 'enable_steering_filter', true);
                addParameter(p, 'steering_filter_alpha', 0.15);
                
                parse(p, varargin{:});
                
                obj.steering_ratio = p.Results.steering_ratio;
                obj.max_steering_wheel_angle = p.Results.max_steering_wheel_angle;
                obj.max_front_wheel_angle = p.Results.max_front_wheel_angle;
                obj.wheelbase = p.Results.wheelbase;
                obj.steering_angle_offset = p.Results.steering_angle_offset;
                obj.steering_angle_gain = p.Results.steering_angle_gain;
                obj.enable_steering_filter = p.Results.enable_steering_filter;
                obj.steering_filter_alpha = p.Results.steering_filter_alpha;
            end
        end
        
        % 方向盘转角转前轮转向角
        function front_wheel_angle = steering_wheel_to_front_wheel(obj, steering_wheel_angle)
            % 转换公式: δ = (α/ratio) * gain + offset
            % 参数:
            %   steering_wheel_angle - 方向盘转角 (rad)
            % 返回:
            %   front_wheel_angle - 前轮转向角 (rad)
            
            % 基本转换
            front_wheel_angle = steering_wheel_angle / obj.steering_ratio;
            
            % 应用校准参数
            front_wheel_angle = obj.apply_calibration(front_wheel_angle);
            
            % 限制在有效范围内
            front_wheel_angle = obj.clamp_angle(front_wheel_angle);
        end
        
        % 前轮转向角转方向盘转角
        function steering_wheel_angle = front_wheel_to_steering_wheel(obj, front_wheel_angle)
            % 逆变换: α = δ * ratio
            % 参数:
            %   front_wheel_angle - 前轮转向角 (rad)
            % 返回:
            %   steering_wheel_angle - 方向盘转角 (rad)
            
            % 逆变换
            steering_wheel_angle = front_wheel_angle * obj.steering_ratio;
            
            % 限制在有效范围内
            if abs(steering_wheel_angle) > obj.max_steering_wheel_angle
                steering_wheel_angle = sign(steering_wheel_angle) * obj.max_steering_wheel_angle;
            end
        end
        
        % 计算航向角速率（偏航角速率）
        function yaw_rate = calculate_yaw_rate(obj, velocity, steering_angle)
            % 基于二轮模型: yaw_rate = v * tan(δ) / L
            % 参数:
            %   velocity - 纵向速度 (m/s)
            %   steering_angle - 转向角 (rad)
            % 返回:
            %   yaw_rate - 航向角速率 (rad/s)
            
            if abs(velocity) < constants.V_STATIC
                yaw_rate = 0;
                return;
            end
            
            % 计算偏航速率
            yaw_rate = velocity * tan(steering_angle) / obj.wheelbase;
        end
        
        % 更新转向角（包括低通滤波）
        function filtered_angle = update(obj, steering_wheel_angle, timestamp)
            % 执行转向角更新和低通滤波
            % 参数:
            %   steering_wheel_angle - 新的方向盘转角 (rad)
            %   timestamp - 时间戳 (s)
            % 返回:
            %   filtered_angle - 滤波后的前轮转向角 (rad)
            
            % 转换为前轮转向角
            front_wheel_angle = obj.steering_wheel_to_front_wheel(steering_wheel_angle);
            
            % 计算时间间隔（如果这是第一次调用，dt=0）
            if obj.prev_timestamp > 0
                dt = timestamp - obj.prev_timestamp;
                if dt > 0
                    obj.steering_rate = (front_wheel_angle - obj.prev_steering_angle) / dt;
                end
            else
                obj.steering_rate = 0;
            end
            
            % 应用低通滤波
            if obj.enable_steering_filter
                alpha = obj.steering_filter_alpha;
                obj.filtered_steering_angle = alpha * obj.filtered_steering_angle + ...
                                              (1 - alpha) * front_wheel_angle;
            else
                obj.filtered_steering_angle = front_wheel_angle;
            end
            
            % 更新前一时刻信息
            obj.prev_steering_angle = front_wheel_angle;
            obj.prev_timestamp = timestamp;
            
            filtered_angle = obj.filtered_steering_angle;
        end
        
        % 应用校准参数
        function calibrated_angle = apply_calibration(obj, angle)
            % 应用零偏和增益补偿
            % 参数:
            %   angle - 原始转向角 (rad)
            % 返回:
            %   calibrated_angle - 校准后的转向角 (rad)
            
            calibrated_angle = (angle - obj.steering_angle_offset) * obj.steering_angle_gain;
        end
        
        % 限制角度在有效范围内
        function clamped_angle = clamp_angle(obj, angle)
            % 限制角度在[-max_angle, +max_angle]范围内
            % 参数:
            %   angle - 原始角度 (rad)
            % 返回:
            %   clamped_angle - 约束后的角度 (rad)
            
            if abs(angle) > obj.max_front_wheel_angle
                clamped_angle = sign(angle) * obj.max_front_wheel_angle;
            else
                clamped_angle = angle;
            end
        end
        
        % 获取当前转向状态
        function state = getState(obj)
            % 返回当前转向模型的状态
            % 返回值为结构体，包含滤波角、角速率等信息
            
            state = struct();
            state.filtered_steering_angle = obj.filtered_steering_angle;
            state.steering_rate = obj.steering_rate;
            state.prev_steering_angle = obj.prev_steering_angle;
        end
        
        % 重置状态
        function obj = reset(obj)
            % 重置转向模型的所有状态变量
            
            obj.filtered_steering_angle = 0.0;
            obj.prev_steering_angle = 0.0;
            obj.steering_rate = 0.0;
            obj.prev_timestamp = 0;
        end
    end
end
