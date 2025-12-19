% constants.m - 车辆定位系统常数定义
% 包含所有系统使用的常数、枚举值和参数

classdef constants
    properties (Constant)
        % 数学常数
        PI = pi                                % 圆周率
        DEG_TO_RAD = pi / 180                 % 度数转弧度系数
        RAD_TO_DEG = 180 / pi                 % 弧度转度数系数
        
        % 速度阈值 (m/s)
        V_STATIC = 0.1                        % 静止速度阈值
        V_LOW_SPEED = 2.0                     % 低速阈值
        V_PARKING_MAX = 0.5                   % 停车最大速度
        
        % 车辆参数
        WHEELBASE = 2.7                       % 车辆轴距 (m)
        
        % 档位枚举
        GEAR_INVALID = -1                     % 无效档位
        GEAR_PARK = 0                         % 停车档
        GEAR_REVERSE = 1                      % 倒车档
        GEAR_NEUTRAL = 2                      % 空档
        GEAR_DRIVE = 3                        % 前进档
        GEAR_LOW = 4                          % 低速档
        
        % 方向枚举
        DIRECTION_REVERSE = -1                % 倒车
        DIRECTION_STATIONARY = 0              % 停止
        DIRECTION_FORWARD = 1                 % 前进
        
        % 融合模式
        FUSION_MODE_BASIC = 0                 % 基础融合模式
        FUSION_MODE_IMU_STEERING = 1          % IMU+转向融合模式
    end
    
    methods (Static)
        % 获取档位名称
        function name = getGearName(gear_value)
            switch gear_value
                case constants.GEAR_PARK
                    name = 'PARK';
                case constants.GEAR_REVERSE
                    name = 'REVERSE';
                case constants.GEAR_NEUTRAL
                    name = 'NEUTRAL';
                case constants.GEAR_DRIVE
                    name = 'DRIVE';
                case constants.GEAR_LOW
                    name = 'LOW';
                otherwise
                    name = 'INVALID';
            end
        end
        
        % 获取方向名称
        function name = getDirectionName(direction_value)
            switch direction_value
                case constants.DIRECTION_FORWARD
                    name = 'FORWARD';
                case constants.DIRECTION_REVERSE
                    name = 'REVERSE';
                case constants.DIRECTION_STATIONARY
                    name = 'STATIONARY';
                otherwise
                    name = 'UNKNOWN';
            end
        end
        
        % 获取融合模式名称
        function name = getFusionModeName(mode_value)
            switch mode_value
                case constants.FUSION_MODE_BASIC
                    name = 'BASIC';
                case constants.FUSION_MODE_IMU_STEERING
                    name = 'IMU_STEERING';
                otherwise
                    name = 'UNKNOWN';
            end
        end
    end
end
