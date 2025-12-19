% CalibrationParams.m - 传感器校准参数类
% 存储和管理各传感器的校准系数和偏差

classdef CalibrationParams
    properties
        % 轮速校准
        wheel_calibration_factor    % 轮速校准系数
        
        % 转向角校准
        steering_angle_offset       % 转向角零点偏移 (rad)
        steering_angle_gain         % 转向角增益系数
        
        % IMU加速度计校准
        imu_accel_bias_x            % X轴加速度零偏 (m/s²)
        imu_accel_bias_y            % Y轴加速度零偏 (m/s²)
        imu_accel_scale_x           % X轴加速度缩放系数
        imu_accel_scale_y           % Y轴加速度缩放系数
        
        % IMU陀螺仪校准
        imu_gyro_bias_z             % Z轴陀螺仪零偏 (rad/s)
    end
    
    methods
        % 构造函数
        function obj = CalibrationParams(varargin)
            % CalibrationParams 构造函数
            % 支持使用默认值或自定义参数
            
            if nargin == 0
                % 默认校准参数值
                obj.wheel_calibration_factor = 1.0;
                obj.steering_angle_offset = 0.0;
                obj.steering_angle_gain = 1.0;
                obj.imu_accel_bias_x = 0.0;
                obj.imu_accel_bias_y = 0.0;
                obj.imu_accel_scale_x = 1.0;
                obj.imu_accel_scale_y = 1.0;
                obj.imu_gyro_bias_z = 0.0;
            else
                % 自定义参数模式
                p = inputParser;
                addParameter(p, 'wheel_calibration_factor', 1.0);
                addParameter(p, 'steering_angle_offset', 0.0);
                addParameter(p, 'steering_angle_gain', 1.0);
                addParameter(p, 'imu_accel_bias_x', 0.0);
                addParameter(p, 'imu_accel_bias_y', 0.0);
                addParameter(p, 'imu_accel_scale_x', 1.0);
                addParameter(p, 'imu_accel_scale_y', 1.0);
                addParameter(p, 'imu_gyro_bias_z', 0.0);
                
                parse(p, varargin{:});
                
                obj.wheel_calibration_factor = p.Results.wheel_calibration_factor;
                obj.steering_angle_offset = p.Results.steering_angle_offset;
                obj.steering_angle_gain = p.Results.steering_angle_gain;
                obj.imu_accel_bias_x = p.Results.imu_accel_bias_x;
                obj.imu_accel_bias_y = p.Results.imu_accel_bias_y;
                obj.imu_accel_scale_x = p.Results.imu_accel_scale_x;
                obj.imu_accel_scale_y = p.Results.imu_accel_scale_y;
                obj.imu_gyro_bias_z = p.Results.imu_gyro_bias_z;
            end
        end
        
        % 应用轮速校准
        function v_calibrated = calibrate_wheel_velocity(obj, v_raw)
            % 对原始轮速进行校准
            v_calibrated = v_raw * obj.wheel_calibration_factor;
        end
        
        % 应用转向角校准
        function delta_calibrated = calibrate_steering_angle(obj, delta_raw)
            % 对原始转向角进行校准
            delta_calibrated = (delta_raw - obj.steering_angle_offset) * obj.steering_angle_gain;
        end
        
        % 应用IMU加速度计校准
        function accel_calibrated = calibrate_imu_accel(obj, accel_x_raw, accel_y_raw, accel_z_raw)
            % 对原始IMU加速度进行校准
            accel_x = (accel_x_raw - obj.imu_accel_bias_x) * obj.imu_accel_scale_x;
            accel_y = (accel_y_raw - obj.imu_accel_bias_y) * obj.imu_accel_scale_y;
            accel_z = accel_z_raw;  % Z轴不做缩放
            
            accel_calibrated = [accel_x; accel_y; accel_z];
        end
        
        % 应用陀螺仪校准
        function gyro_z_calibrated = calibrate_imu_gyro(obj, gyro_z_raw)
            % 对原始陀螺仪进行校准
            gyro_z_calibrated = gyro_z_raw - obj.imu_gyro_bias_z;
        end
        
        % 获取校准参数向量
        function calib_vec = getCalibrationVector(obj)
            % 返回所有校准参数为列向量
            calib_vec = [
                obj.wheel_calibration_factor;
                obj.steering_angle_offset;
                obj.steering_angle_gain;
                obj.imu_accel_bias_x;
                obj.imu_accel_bias_y;
                obj.imu_accel_scale_x;
                obj.imu_accel_scale_y;
                obj.imu_gyro_bias_z
            ];
        end
        
        % 深拷贝
        function obj_copy = copy(obj)
            % 创建当前对象的深拷贝
            obj_copy = CalibrationParams(...
                'wheel_calibration_factor', obj.wheel_calibration_factor, ...
                'steering_angle_offset', obj.steering_angle_offset, ...
                'steering_angle_gain', obj.steering_angle_gain, ...
                'imu_accel_bias_x', obj.imu_accel_bias_x, ...
                'imu_accel_bias_y', obj.imu_accel_bias_y, ...
                'imu_accel_scale_x', obj.imu_accel_scale_x, ...
                'imu_accel_scale_y', obj.imu_accel_scale_y, ...
                'imu_gyro_bias_z', obj.imu_gyro_bias_z ...
            );
        end
        
        % 显示校准参数
        function display(obj)
            fprintf('CalibrationParams:\n');
            fprintf('  Wheel Calibration Factor: %.6f\n', obj.wheel_calibration_factor);
            fprintf('  Steering Angle Offset: %.6f rad (%.2f deg)\n', ...
                    obj.steering_angle_offset, obj.steering_angle_offset * constants.RAD_TO_DEG);
            fprintf('  Steering Angle Gain: %.6f\n', obj.steering_angle_gain);
            fprintf('  IMU Accel Bias (X,Y): (%.6f, %.6f) m/s²\n', ...
                    obj.imu_accel_bias_x, obj.imu_accel_bias_y);
            fprintf('  IMU Accel Scale (X,Y): (%.6f, %.6f)\n', ...
                    obj.imu_accel_scale_x, obj.imu_accel_scale_y);
            fprintf('  IMU Gyro Bias Z: %.6f rad/s\n', obj.imu_gyro_bias_z);
        end
    end
end
