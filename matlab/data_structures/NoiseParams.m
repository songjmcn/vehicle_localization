% NoiseParams.m - 卡尔曼滤波器噪声参数类
% 管理系统噪声协方差矩阵和滤波器参数

classdef NoiseParams
    properties
        % 过程噪声 (Q矩阵对角线元素)
        q_x             % 位置X噪声方差
        q_y             % 位置Y噪声方差
        q_theta         % 航向角噪声方差
        q_v             % 速度噪声方差
        q_delta         % 转向角噪声方差
        
        % 测量噪声 (R矩阵对角线元素)
        r_wheel         % 轮速测量噪声方差
        r_imu           % IMU基础测量噪声方差
        r_steering      % 转向角测量噪声方差
        r_imu_steering  % IMU+转向融合测量噪声方差
        
        % 转向角滤波参数
        filter_alpha    % 转向角低通滤波器系数 (0-1)
        jitter_threshold % 转向角抖动阈值 (rad)
    end
    
    methods
        % 构造函数
        function obj = NoiseParams(varargin)
            % NoiseParams 构造函数
            % 可选参数支持自定义所有参数，或使用预设值
            
            % 默认参数值（基于工程经验）
            if nargin == 0
                % 过程噪声默认值
                obj.q_x = 0.01;
                obj.q_y = 0.01;
                obj.q_theta = 0.001;
                obj.q_v = 0.1;
                obj.q_delta = 0.01;
                
                % 测量噪声默认值
                obj.r_wheel = 0.1;
                obj.r_imu = 0.5;
                obj.r_steering = 0.01;
                obj.r_imu_steering = 0.05;
                
                % 滤波器参数默认值
                obj.filter_alpha = 0.3;
                obj.jitter_threshold = 0.005;
            else
                % 自定义参数模式
                p = inputParser;
                addParameter(p, 'q_x', 0.01);
                addParameter(p, 'q_y', 0.01);
                addParameter(p, 'q_theta', 0.001);
                addParameter(p, 'q_v', 0.1);
                addParameter(p, 'q_delta', 0.01);
                addParameter(p, 'r_wheel', 0.1);
                addParameter(p, 'r_imu', 0.5);
                addParameter(p, 'r_steering', 0.01);
                addParameter(p, 'r_imu_steering', 0.05);
                addParameter(p, 'filter_alpha', 0.3);
                addParameter(p, 'jitter_threshold', 0.005);
                
                parse(p, varargin{:});
                
                obj.q_x = p.Results.q_x;
                obj.q_y = p.Results.q_y;
                obj.q_theta = p.Results.q_theta;
                obj.q_v = p.Results.q_v;
                obj.q_delta = p.Results.q_delta;
                obj.r_wheel = p.Results.r_wheel;
                obj.r_imu = p.Results.r_imu;
                obj.r_steering = p.Results.r_steering;
                obj.r_imu_steering = p.Results.r_imu_steering;
                obj.filter_alpha = p.Results.filter_alpha;
                obj.jitter_threshold = p.Results.jitter_threshold;
            end
        end
        
        % 获取Q矩阵 (过程噪声协方差)
        function Q = getQMatrix(obj)
            % 返回5x5过程噪声协方差矩阵
            Q = diag([obj.q_x, obj.q_y, obj.q_theta, obj.q_v, obj.q_delta]);
        end
        
        % 获取R矩阵 (测量噪声协方差) - 基础模式
        function R = getRMatrixBasic(obj)
            % 返回基础融合模式的R矩阵
            % 包含：轮速、转向角
            R = diag([obj.r_wheel, obj.r_steering]);
        end
        
        % 获取R矩阵 - IMU+转向融合模式
        function R = getRMatrixIMUSteering(obj)
            % 返回IMU+转向融合模式的R矩阵
            % 包含：轮速、IMU测量、转向角
            R = diag([obj.r_wheel, obj.r_imu, obj.r_imu_steering]);
        end
        
        % 设置过程噪声参数
        function obj = setProcessNoise(obj, q_x, q_y, q_theta, q_v, q_delta)
            % 批量设置过程噪声参数
            if nargin == 6
                obj.q_x = q_x;
                obj.q_y = q_y;
                obj.q_theta = q_theta;
                obj.q_v = q_v;
                obj.q_delta = q_delta;
            end
        end
        
        % 设置测量噪声参数
        function obj = setMeasurementNoise(obj, r_wheel, r_imu, r_steering, r_imu_steering)
            % 批量设置测量噪声参数
            if nargin == 5
                obj.r_wheel = r_wheel;
                obj.r_imu = r_imu;
                obj.r_steering = r_steering;
                obj.r_imu_steering = r_imu_steering;
            end
        end
        
        % 显示噪声参数
        function display(obj)
            fprintf('NoiseParams:\n');
            fprintf('  Process Noise (Q):\n');
            fprintf('    q_x: %.4f\n', obj.q_x);
            fprintf('    q_y: %.4f\n', obj.q_y);
            fprintf('    q_theta: %.6f\n', obj.q_theta);
            fprintf('    q_v: %.4f\n', obj.q_v);
            fprintf('    q_delta: %.4f\n', obj.q_delta);
            fprintf('  Measurement Noise (R):\n');
            fprintf('    r_wheel: %.4f\n', obj.r_wheel);
            fprintf('    r_imu: %.4f\n', obj.r_imu);
            fprintf('    r_steering: %.4f\n', obj.r_steering);
            fprintf('    r_imu_steering: %.4f\n', obj.r_imu_steering);
            fprintf('  Filter Parameters:\n');
            fprintf('    filter_alpha: %.3f\n', obj.filter_alpha);
            fprintf('    jitter_threshold: %.6f rad\n', obj.jitter_threshold);
        end
    end
end
