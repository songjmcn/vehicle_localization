% SensorData.m - 传感器数据容器类
% 存储和处理来自各传感器的原始测量数据

classdef SensorData
    properties
        wheel_velocity      % 轮速 (m/s)
        imu_accel_x         % IMU加速度 X轴分量 (m/s²)
        imu_accel_y         % IMU加速度 Y轴分量 (m/s²)
        imu_accel_z         % IMU加速度 Z轴分量 (m/s²)
        imu_gyro_z          % IMU陀螺仪 Z轴角速度 (rad/s)
        steering_angle      % 转向角 (rad)
        timestamp           % 时间戳 (s)
    end
    
    methods
        % 构造函数
        function obj = SensorData(wheel_velocity, imu_accel_x, imu_accel_y, ...
                                  imu_accel_z, imu_gyro_z, steering_angle, timestamp)
            % SensorData 构造函数
            
            if nargin == 7
                obj.wheel_velocity = wheel_velocity;
                obj.imu_accel_x = imu_accel_x;
                obj.imu_accel_y = imu_accel_y;
                obj.imu_accel_z = imu_accel_z;
                obj.imu_gyro_z = imu_gyro_z;
                obj.steering_angle = steering_angle;
                obj.timestamp = timestamp;
            else
                % 默认初始化
                obj.wheel_velocity = 0;
                obj.imu_accel_x = 0;
                obj.imu_accel_y = 0;
                obj.imu_accel_z = 0;
                obj.imu_gyro_z = 0;
                obj.steering_angle = 0;
                obj.timestamp = 0;
            end
        end
        
        % 获取加速度向量
        function accel = get_accel_vector(obj)
            % 返回IMU加速度向量 [ax, ay, az]'
            accel = [obj.imu_accel_x; obj.imu_accel_y; obj.imu_accel_z];
        end
        
        % 获取加速度大小
        function magnitude = get_accel_magnitude(obj)
            % 计算加速度的模: sqrt(ax² + ay² + az²)
            magnitude = sqrt(obj.imu_accel_x^2 + obj.imu_accel_y^2 + obj.imu_accel_z^2);
        end
        
        % 获取横向加速度
        function accel_lateral = get_accel_lateral(obj)
            % 返回IMU横向加速度 (Y轴)
            accel_lateral = obj.imu_accel_y;
        end
        
        % 获取纵向加速度
        function accel_longitudinal = get_accel_longitudinal(obj)
            % 返回IMU纵向加速度 (X轴)
            accel_longitudinal = obj.imu_accel_x;
        end
        
        % 获取角速度 (Z轴)
        function gyro_z = get_gyro_z(obj)
            % 返回IMU陀螺仪Z轴角速度
            gyro_z = obj.imu_gyro_z;
        end
        
        % 深拷贝
        function obj_copy = copy(obj)
            % 创建当前对象的深拷贝
            obj_copy = SensorData(obj.wheel_velocity, obj.imu_accel_x, obj.imu_accel_y, ...
                                  obj.imu_accel_z, obj.imu_gyro_z, obj.steering_angle, obj.timestamp);
        end
        
        % 显示传感器数据
        function display(obj)
            fprintf('SensorData:\n');
            fprintf('  Wheel Velocity: %.3f m/s\n', obj.wheel_velocity);
            fprintf('  IMU Accel: (%.3f, %.3f, %.3f) m/s²\n', ...
                    obj.imu_accel_x, obj.imu_accel_y, obj.imu_accel_z);
            fprintf('  Accel Magnitude: %.3f m/s²\n', obj.get_accel_magnitude());
            fprintf('  IMU Gyro Z: %.3f rad/s\n', obj.imu_gyro_z);
            fprintf('  Steering Angle: %.3f rad (%.1f deg)\n', ...
                    obj.steering_angle, obj.steering_angle * constants.RAD_TO_DEG);
            fprintf('  Timestamp: %.3f s\n', obj.timestamp);
        end
    end
end
