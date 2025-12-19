% MotionModel.m - 单轨运动学模型类
% 实现车辆运动预测和雅可比矩阵计算

classdef MotionModel
    properties
        wheelbase = constants.WHEELBASE         % 轴距 (m)
        v_static_threshold = constants.V_STATIC % 静止速度阈值 (m/s)
        max_steering_angle = 0.628              % 最大转向角 (rad)
    end
    
    methods
        % 构造函数
        function obj = MotionModel(varargin)
            % MotionModel 构造函数
            % 可选参数: 'wheelbase', 'v_static_threshold', 'max_steering_angle'
            
            if nargin > 0
                p = inputParser;
                addParameter(p, 'wheelbase', constants.WHEELBASE);
                addParameter(p, 'v_static_threshold', constants.V_STATIC);
                addParameter(p, 'max_steering_angle', 0.628);
                
                parse(p, varargin{:});
                
                obj.wheelbase = p.Results.wheelbase;
                obj.v_static_threshold = p.Results.v_static_threshold;
                obj.max_steering_angle = p.Results.max_steering_angle;
            end
        end
        
        % 单轨运动模型预测
        function new_state = predict(obj, state, velocity, steering_angle, dt)
            % 使用单轨模型预测车辆状态
            % 参数:
            %   state - VehicleState 对象
            %   velocity - 速度 (m/s)
            %   steering_angle - 转向角 (rad)
            %   dt - 时间间隔 (s)
            % 返回:
            %   new_state - 预测后的 VehicleState 对象
            
            % 创建新状态对象
            new_state = state.copy();
            
            % 检查是否静止
            if abs(velocity) < obj.v_static_threshold
                % 完全静止，位置和角度不变
                new_state.x = state.x;
                new_state.y = state.y;
                new_state.theta = state.theta;
                new_state.v = 0.0;
                new_state.delta = steering_angle;
                new_state.direction = constants.DIRECTION_STATIONARY;
                return;
            end
            
            % 推导运动方向
            direction = obj.derive_direction(state.gear, velocity);
            sign_v = 1;
            if direction == constants.DIRECTION_REVERSE
                sign_v = -1;
            end
            
            % 转向角在后退时反向
            abs_v = abs(velocity);
            effective_delta = sign_v * steering_angle;
            
            % 位置更新
            cos_term = cos(state.theta + effective_delta);
            sin_term = sin(state.theta + effective_delta);
            
            new_state.x = state.x + abs_v * cos_term * dt;
            new_state.y = state.y + abs_v * sin_term * dt;
            
            % 航向角更新
            tan_delta = tan(effective_delta);
            new_state.theta = state.theta + (sign_v * abs_v * tan_delta / obj.wheelbase) * dt;
            new_state.theta = angle_utils.normalize_angle(new_state.theta);
            
            new_state.v = velocity;
            new_state.delta = steering_angle;
            new_state.direction = direction;
        end
        
        % 计算状态转移雅可比矩阵
        function F = jacobian(obj, state, velocity, steering_angle, dt)
            % 计算运动模型的状态转移雅可比矩阵
            % 用于卡尔曼滤波预测步骤
            % 参数:
            %   state - 当前 VehicleState 对象
            %   velocity - 速度 (m/s)
            %   steering_angle - 转向角 (rad)
            %   dt - 时间间隔 (s)
            % 返回:
            %   F - 5x5 雅可比矩阵
            
            % 初始化为单位矩阵
            F = eye(5);
            
            % 如果速度太小，返回单位矩阵
            if abs(velocity) < obj.v_static_threshold
                return;
            end
            
            theta = state.theta;
            
            % 推导运动方向
            direction = obj.derive_direction(state.gear, velocity);
            sign_v = 1;
            if direction == constants.DIRECTION_REVERSE
                sign_v = -1;
            end
            
            abs_v = abs(velocity);
            effective_delta = sign_v * steering_angle;
            
            cos_term = cos(theta + effective_delta);
            sin_term = sin(theta + effective_delta);
            cos_delta = cos(effective_delta);
            
            % ∂x/∂θ
            F(1, 3) = -abs_v * sin_term * dt;
            % ∂y/∂θ
            F(2, 3) = abs_v * cos_term * dt;
            
            % ∂x/∂v
            F(1, 4) = sign_v * cos_term * dt;
            % ∂y/∂v
            F(2, 4) = sign_v * sin_term * dt;
            
            % ∂θ/∂v 和 ∂θ/∂δ
            if abs(cos_delta) > 1e-6
                sec_delta_sq = 1.0 / (cos_delta^2);
                
                % ∂θ/∂v
                F(3, 4) = sign_v * (tan(effective_delta) / obj.wheelbase) * dt;
                
                % ∂θ/∂δ
                F(3, 5) = (sign_v * abs_v * sec_delta_sq / obj.wheelbase) * dt;
            end
        end
        
        % 推导运动方向
        function direction = derive_direction(obj, gear, velocity)
            % 从档位和速度推导运动方向
            % 参数:
            %   gear - 车辆档位
            %   velocity - 速度 (m/s)
            % 返回:
            %   direction - DIRECTION_FORWARD, DIRECTION_REVERSE, 或 DIRECTION_STATIONARY
            
            if abs(velocity) < obj.v_static_threshold
                direction = constants.DIRECTION_STATIONARY;
                return;
            end
            
            % 如果有有效档位信息，优先使用
            if gear == constants.GEAR_REVERSE
                direction = constants.DIRECTION_REVERSE;
            elseif gear == constants.GEAR_DRIVE || gear == constants.GEAR_LOW
                direction = constants.DIRECTION_FORWARD;
            else
                % 后备方案：仅基于速度信号
                if velocity > 0
                    direction = constants.DIRECTION_FORWARD;
                else
                    direction = constants.DIRECTION_REVERSE;
                end
            end
        end
    end
end
