% FusionConfig.m - 传感器融合配置类
% 用于配置和管理多传感器融合算法参数

classdef FusionConfig
    properties
        mode                            % 融合模式 (BASIC 或 IMU_STEERING)
        imu_steering_vel_threshold      % IMU转向推导速度下限 (m/s)
        imu_steering_weight             % IMU转向融合权重 [0,1]
        enable_adaptive_weighting       % 是否启用自适应权重调整
    end
    
    methods
        % 构造函数
        function obj = FusionConfig(varargin)
            % FusionConfig 构造函数
            
            if nargin == 0
                % 默认配置：基础融合模式
                obj.mode = constants.FUSION_MODE_BASIC;
                obj.imu_steering_vel_threshold = 0.5;
                obj.imu_steering_weight = 0.5;
                obj.enable_adaptive_weighting = false;
            else
                % 自定义配置
                p = inputParser;
                addParameter(p, 'mode', constants.FUSION_MODE_BASIC);
                addParameter(p, 'imu_steering_vel_threshold', 0.5);
                addParameter(p, 'imu_steering_weight', 0.5);
                addParameter(p, 'enable_adaptive_weighting', false);
                
                parse(p, varargin{:});
                
                obj.mode = p.Results.mode;
                obj.imu_steering_vel_threshold = p.Results.imu_steering_vel_threshold;
                obj.imu_steering_weight = p.Results.imu_steering_weight;
                obj.enable_adaptive_weighting = p.Results.enable_adaptive_weighting;
            end
        end
        
        % 切换融合模式
        function obj = setMode(obj, mode)
            % 设置融合模式
            % mode: constants.FUSION_MODE_BASIC 或 constants.FUSION_MODE_IMU_STEERING
            if mode == constants.FUSION_MODE_BASIC || mode == constants.FUSION_MODE_IMU_STEERING
                obj.mode = mode;
            else
                error('无效的融合模式');
            end
        end
        
        % 检查是否启用IMU融合
        function enabled = isIMUSteering(obj)
            % 返回是否启用IMU+转向融合模式
            enabled = (obj.mode == constants.FUSION_MODE_IMU_STEERING);
        end
        
        % 计算自适应权重
        function weight = getAdaptiveWeight(obj, velocity)
            % 基于速度计算自适应权重
            % 低速时增加IMU融合权重，高速时降低
            
            if ~obj.enable_adaptive_weighting
                weight = obj.imu_steering_weight;
                return;
            end
            
            % 自适应权重公式
            v_threshold = obj.imu_steering_vel_threshold;
            if velocity < v_threshold
                % 低速：增加IMU权重
                weight = obj.imu_steering_weight * (1.5 - 0.5 * velocity / v_threshold);
            else
                % 高速：降低IMU权重
                weight = obj.imu_steering_weight * 0.5;
            end
            
            % 限制在 [0, 1]
            weight = max(0, min(1, weight));
        end
        
        % 深拷贝
        function obj_copy = copy(obj)
            % 创建当前对象的深拷贝
            obj_copy = FusionConfig(...
                'mode', obj.mode, ...
                'imu_steering_vel_threshold', obj.imu_steering_vel_threshold, ...
                'imu_steering_weight', obj.imu_steering_weight, ...
                'enable_adaptive_weighting', obj.enable_adaptive_weighting ...
            );
        end
        
        % 显示融合配置
        function display(obj)
            fprintf('FusionConfig:\n');
            fprintf('  Mode: %s\n', constants.getFusionModeName(obj.mode));
            fprintf('  IMU Steering Velocity Threshold: %.3f m/s\n', obj.imu_steering_vel_threshold);
            fprintf('  IMU Steering Weight: %.3f\n', obj.imu_steering_weight);
            fprintf('  Adaptive Weighting: %s\n', mat2str(logical(obj.enable_adaptive_weighting)));
        end
    end
end
