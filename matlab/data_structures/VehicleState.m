% VehicleState.m - 车辆状态表示类
% 存储和管理车辆的位置、速度、方向等状态信息

classdef VehicleState
    properties
        x               % 全局X坐标 (m)
        y               % 全局Y坐标 (m)
        theta           % 航向角 (rad)
        v               % 纵向速度 (m/s)
        delta           % 转向角 (rad)
        timestamp       % 时间戳 (s)
        gear            % 当前档位
        direction       % 运动方向 (-1: 倒车, 0: 停止, 1: 前进)
    end
    
    methods
        % 构造函数
        function obj = VehicleState(x, y, theta, v, delta, timestamp, gear, direction)
            % VehicleState 构造函数
            % 参数:
            %   x        - X坐标 (m)
            %   y        - Y坐标 (m)
            %   theta    - 航向角 (rad)
            %   v        - 纵向速度 (m/s)
            %   delta    - 转向角 (rad)
            %   timestamp - 时间戳 (s)
            %   gear     - 档位值
            %   direction - 方向 (-1/0/1)
            
            if nargin == 8
                obj.x = x;
                obj.y = y;
                obj.theta = theta;
                obj.v = v;
                obj.delta = delta;
                obj.timestamp = timestamp;
                obj.gear = gear;
                obj.direction = direction;
            else
                % 默认初始化
                obj.x = 0;
                obj.y = 0;
                obj.theta = 0;
                obj.v = 0;
                obj.delta = 0;
                obj.timestamp = 0;
                obj.gear = constants.GEAR_NEUTRAL;
                obj.direction = constants.DIRECTION_STATIONARY;
            end
            
            % 规范化角度
            obj.theta = angle_utils.normalize_angle(obj.theta);
        end
        
        % 转换为向量形式
        function vec = toVector(obj)
            % 返回状态的列向量表示 [x, y, theta, v, delta]'
            vec = [obj.x; obj.y; obj.theta; obj.v; obj.delta];
        end
        
        % 从向量恢复状态
        function obj = fromVector(obj, vec)
            % 从列向量恢复状态
            % 参数:
            %   vec - 5元素列向量 [x, y, theta, v, delta]'
            
            if length(vec) ~= 5
                error('向量长度必须为5');
            end
            
            obj.x = vec(1);
            obj.y = vec(2);
            obj.theta = angle_utils.normalize_angle(vec(3));
            obj.v = vec(4);
            obj.delta = vec(5);
        end
        
        % 深拷贝
        function obj_copy = copy(obj)
            % 创建当前对象的深拷贝
            obj_copy = VehicleState(obj.x, obj.y, obj.theta, obj.v, ...
                                    obj.delta, obj.timestamp, obj.gear, obj.direction);
        end
        
        % 规范化航向角
        function obj = normalize_angle(obj)
            % 将航向角规范化到 [-π, π]
            obj.theta = angle_utils.normalize_angle(obj.theta);
        end
        
        % 获取位置向量
        function pos = getPosition(obj)
            % 返回 [x, y] 位置向量
            pos = [obj.x; obj.y];
        end
        
        % 设置位置
        function obj = setPosition(obj, x, y)
            % 设置 X 和 Y 坐标
            obj.x = x;
            obj.y = y;
        end
        
        % 显示状态信息
        function display(obj)
            fprintf('VehicleState:\n');
            fprintf('  Position: (%.3f, %.3f) m\n', obj.x, obj.y);
            fprintf('  Heading: %.3f rad (%.1f deg)\n', obj.theta, obj.theta * constants.RAD_TO_DEG);
            fprintf('  Velocity: %.3f m/s\n', obj.v);
            fprintf('  Steering Angle: %.3f rad (%.1f deg)\n', obj.delta, obj.delta * constants.RAD_TO_DEG);
            fprintf('  Timestamp: %.3f s\n', obj.timestamp);
            fprintf('  Gear: %s\n', constants.getGearName(obj.gear));
            fprintf('  Direction: %s\n', constants.getDirectionName(obj.direction));
        end
    end
end
