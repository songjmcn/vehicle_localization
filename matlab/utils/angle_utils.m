% angle_utils.m - 角度处理工具函数模块 (非类定义)
% 提供角度规范化、转换等纯函数功能

classdef angle_utils
    methods (Static)
        % 规范化角度到 [-π, π]
        function normalized = normalize_angle(angle)
            % 将任意角度规范化到 [-π, π] 范围内
            % 参数:
            %   angle - 输入角度 (rad)
            % 返回:
            %   normalized - 规范化后的角度 (rad)
            
            % 使用atan2进行规范化
            normalized = atan2(sin(angle), cos(angle));
        end
        
        % 计算两个角度之间的最小差值
        function diff = angle_difference(a1, a2)
            % 计算a1和a2之间的最小有向角差
            % 返回值在 [-π, π] 范围内
            % 参数:
            %   a1, a2 - 两个角度 (rad)
            % 返回:
            %   diff - 角度差 (rad), a1 - a2的最小值
            
            % 计算原始差值
            diff = a1 - a2;
            
            % 规范化到 [-π, π]
            diff = angle_utils.normalize_angle(diff);
        end
        
        % 度转弧度
        function rad = deg_to_rad(deg)
            % 将角度从度数转换为弧度
            % 参数:
            %   deg - 度数
            % 返回:
            %   rad - 弧度
            rad = deg * constants.DEG_TO_RAD;
        end
        
        % 弧度转度
        function deg = rad_to_deg(rad)
            % 将角度从弧度转换为度数
            % 参数:
            %   rad - 弧度
            % 返回:
            %   deg - 度数
            deg = rad * constants.RAD_TO_DEG;
        end
        
        % 线性插值角度
        function interpolated = angle_lerp(a1, a2, t)
            % 对两个角度进行线性插值
            % 使用球面线性插值方法处理角度环绕
            % 参数:
            %   a1 - 起始角度 (rad)
            %   a2 - 结束角度 (rad)
            %   t  - 插值系数 (0-1, 0=a1, 1=a2)
            % 返回:
            %   interpolated - 插值后的角度 (rad)
            
            % 计算角度差（最小路径）
            diff = angle_utils.angle_difference(a2, a1);
            
            % 线性插值
            interpolated = a1 + t * diff;
            
            % 规范化结果
            interpolated = angle_utils.normalize_angle(interpolated);
        end
        
        % 检查角度是否在范围内
        function in_range = is_angle_in_range(angle, center, half_width)
            % 检查角度是否在center±half_width的范围内
            % 参数:
            %   angle      - 要检查的角度 (rad)
            %   center     - 范围中心 (rad)
            %   half_width - 范围半宽 (rad)
            % 返回:
            %   in_range - 逻辑值 (true/false)
            
            diff = abs(angle_utils.angle_difference(angle, center));
            in_range = diff <= half_width;
        end
        
        % 将角度限制在范围内
        function constrained = constrain_angle(angle, min_angle, max_angle)
            % 将角度限制在[min_angle, max_angle]范围内
            % 注意：此函数假设范围跨度不超过2π
            % 参数:
            %   angle     - 输入角度 (rad)
            %   min_angle - 最小角度 (rad)
            %   max_angle - 最大角度 (rad)
            % 返回:
            %   constrained - 约束后的角度 (rad)
            
            % 规范化输入
            angle = angle_utils.normalize_angle(angle);
            min_angle = angle_utils.normalize_angle(min_angle);
            max_angle = angle_utils.normalize_angle(max_angle);
            
            if min_angle <= max_angle
                % 正常情况
                if angle < min_angle
                    constrained = min_angle;
                elseif angle > max_angle
                    constrained = max_angle;
                else
                    constrained = angle;
                end
            else
                % 跨越 ±π 的情况
                if angle > max_angle && angle < min_angle
                    % 在两个范围之间，选择最近的一个
                    if abs(angle - min_angle) < abs(angle - max_angle)
                        constrained = min_angle;
                    else
                        constrained = max_angle;
                    end
                else
                    constrained = angle;
                end
            end
        end
        
        % 计算角速度（角度变化率）
        function omega = angle_velocity(angle_1, angle_2, dt)
            % 基于两个时刻的角度计算角速度
            % 参数:
            %   angle_1 - t1时刻的角度 (rad)
            %   angle_2 - t2时刻的角度 (rad)
            %   dt      - 时间间隔 (s)
            % 返回:
            %   omega - 角速度 (rad/s)
            
            if dt <= 0
                error('时间间隔dt必须大于0');
            end
            
            angle_diff = angle_utils.angle_difference(angle_2, angle_1);
            omega = angle_diff / dt;
        end
    end
end
