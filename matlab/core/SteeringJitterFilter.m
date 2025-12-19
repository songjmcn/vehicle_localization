% SteeringJitterFilter.m - 转向角抖动过滤器类
% 过滤转向角传感器噪声，计算零偏角

classdef SteeringJitterFilter
    properties
        angle_history = []              % 转向角历史缓冲
        max_history_size = 10           % 最大历史大小
        history_index = 1               % 循环缓冲索引
        
        % 统计量
        mean_angle = 0.0                % 转向角均值
        angle_variance = 0.0            % 转向角方差
        zero_bias_angle = 0.0           % 零偏角估计
        
        % 过滤参数
        jitter_threshold = 0.005        % 抖动阈值 (rad)
        filter_alpha = 0.15             % 低通滤波系数
        is_high_jitter = false          % 高抖动标志
    end
    
    methods
        % 构造函数
        function obj = SteeringJitterFilter(varargin)
            % SteeringJitterFilter 构造函数
            % 参数: 'jitter_threshold', 'filter_alpha', 'history_size'
            
            if nargin > 0
                p = inputParser;
                addParameter(p, 'jitter_threshold', 0.005);
                addParameter(p, 'filter_alpha', 0.15);
                addParameter(p, 'history_size', 10);
                
                parse(p, varargin{:});
                
                obj.jitter_threshold = p.Results.jitter_threshold;
                obj.filter_alpha = p.Results.filter_alpha;
                obj.max_history_size = p.Results.history_size;
            end
            
            % 初始化历史缓冲
            obj.angle_history = zeros(obj.max_history_size, 1);
        end
        
        % 更新过滤器
        function obj = update(obj, steering_angle)
            % 将新角度添加到历史缓冲并更新统计量
            % 参数:
            %   steering_angle - 新的转向角 (rad)
            
            % 添加到循环缓冲
            obj.angle_history(obj.history_index) = steering_angle;
            
            % 更新索引
            obj.history_index = mod(obj.history_index, obj.max_history_size) + 1;
            
            % 计算统计量
            obj.mean_angle = mean(obj.angle_history);
            obj.angle_variance = var(obj.angle_history);
            
            % 判断是否高抖动
            obj.is_high_jitter = (obj.angle_variance > obj.jitter_threshold^2);
        end
        
        % 计算零偏
        function [success, offset] = compute_offset(obj, velocity)
            % 仅在低速或静止时更新零偏
            % 参数:
            %   velocity - 当前速度 (m/s)
            % 返回:
            %   success - 是否成功计算（逻辑值）
            %   offset - 计算出的零偏角 (rad)
            
            % 低速阈值
            v_threshold = 0.3;  % m/s
            
            % 检查是否可以更新零偏
            if abs(velocity) < v_threshold && ~obj.is_high_jitter
                % 低速且非高抖动，更新零偏
                obj.zero_bias_angle = obj.mean_angle;
                success = true;
                offset = obj.zero_bias_angle;
            else
                % 不更新零偏
                success = false;
                offset = obj.zero_bias_angle;
            end
        end
        
        % 应用低通滤波
        function filtered_angle = apply_filter(obj, steering_angle)
            % 对输入角度应用一阶低通滤波
            % 参数:
            %   steering_angle - 输入转向角 (rad)
            % 返回:
            %   filtered_angle - 滤波后的转向角 (rad)
            
            alpha = obj.filter_alpha;
            filtered_angle = alpha * obj.mean_angle + (1 - alpha) * steering_angle;
        end
        
        % 获取统计信息
        function stats = get_stats(obj)
            % 返回统计信息结构体
            % 返回值包含:
            %   mean - 平均值
            %   variance - 方差
            %   std - 标准差
            %   zero_bias - 零偏
            %   is_high_jitter - 高抖动标志
            
            stats = struct();
            stats.mean = obj.mean_angle;
            stats.variance = obj.angle_variance;
            stats.std = sqrt(obj.angle_variance);
            stats.zero_bias = obj.zero_bias_angle;
            stats.is_high_jitter = obj.is_high_jitter;
        end
        
        % 重置过滤器
        function obj = reset(obj)
            % 重置所有状态
            obj.angle_history = zeros(obj.max_history_size, 1);
            obj.history_index = 1;
            obj.mean_angle = 0.0;
            obj.angle_variance = 0.0;
            obj.zero_bias_angle = 0.0;
            obj.is_high_jitter = false;
        end
        
        % 显示状态
        function display(obj)
            fprintf('SteeringJitterFilter:\n');
            fprintf('  Mean Angle: %.4f rad (%.2f deg)\n', ...
                    obj.mean_angle, obj.mean_angle * constants.RAD_TO_DEG);
            fprintf('  Angle Variance: %.6f rad²\n', obj.angle_variance);
            fprintf('  Std Dev: %.4f rad (%.2f deg)\n', ...
                    sqrt(obj.angle_variance), sqrt(obj.angle_variance) * constants.RAD_TO_DEG);
            fprintf('  Zero Bias: %.4f rad (%.2f deg)\n', ...
                    obj.zero_bias_angle, obj.zero_bias_angle * constants.RAD_TO_DEG);
            fprintf('  Jitter Threshold: %.6f rad\n', obj.jitter_threshold);
            fprintf('  High Jitter: %s\n', mat2str(logical(obj.is_high_jitter)));
            fprintf('  Filter Alpha: %.3f\n', obj.filter_alpha);
        end
    end
end
