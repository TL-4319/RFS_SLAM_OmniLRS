close all
clear
clc

addpath ../utils/

use_noisy = 1;

%% This script read the recorded bags and convert them to csv type files for processing
dataset_name = 'straight';
%dataset_name = 'straight_turn_straight';

path_to_folder = horzcat('datasets/preprocess/', dataset_name,'/');
load("blickfeld_cube1_400_scan_pattern.mat");
scan_pattern.FOV_bounds = deg2rad(scan_pattern.FOV_bounds);
dataset.scan_pattern = scan_pattern;
sensor_param.meas_model = 'cartesian';

%% Read timing information
timing_data = readmatrix(strcat(path_to_folder,'timing.csv'));

dataset.time_vec = (timing_data(:,1) - timing_data(1,1))/1000;
dataset.lidar_avail = timing_data(:,2);
dataset.sensor_time_vec = dataset.time_vec(dataset.lidar_avail == 1);

peak_measurement_vec = cell(size(dataset.sensor_time_vec,1),1);
crater_measurement_vec = cell(size(dataset.sensor_time_vec,1),1);

dt = dataset.time_vec(2) - dataset.time_vec(1);

%% Read pose and odom
[~, ENU_tf_world_base] = file2se3(strcat(path_to_folder,'base_world_tf.csv')); % ENU
[~, ENU_tf_base_mastMount] = file2se3(horzcat(path_to_folder,'mast_mount_base_tf.csv'));
[~, ENU_tf_mastMount_sensorMount] = file2se3(horzcat(path_to_folder,'sensor_mount_mast_mount_tf.csv'));
[~, ENU_tf_sensorMount_lidar]= file2se3(horzcat(path_to_folder,'lidar_sensor_mount_tf.csv'));

% Conversion from blickfeld frame(ENU) to isaac lidar frame (NWU) for
% synthetic data processing
tf_lidar_isaacLidar = eye(4);
tf_lidar_isaacLidar(1:2, 1:2) = [0 -1; 1 0];

% Converstion from ENU to NED
tf_NED_ENU = eye(4);
tf_NED_ENU(1:3, 1:3) = [0 1 0;
                        1 0 0;
                        0 0 -1];
tf_ENU_NED = tf_NED_ENU; % Same inverse

% Convertsion from NED to isaacLidar
tf_isaacLidar_NED = eye(4);
tf_isaacLidar_NED(1:3,1:3)  = [1 0 0;
                               0 -1 0;
                               0 0 -1];
tf_NED_isaacLidar = tf_isaacLidar_NED;

% Process odom and convert to NED
ENU_odom = readmatrix(strcat(path_to_folder,'odom.csv'));
ENU_body_trans_vel = ENU_odom(:,5:7);
ENU_body_rot_vel = ENU_odom(:,2:4);

NED_body_trans_vel = apply_transform(tf_NED_ENU, ENU_body_trans_vel');
dataset.trans_vel_body = NED_body_trans_vel;
NED_body_rot_vel = apply_transform(tf_NED_ENU, ENU_body_rot_vel');
dataset.rot_vel_body = NED_body_rot_vel;

NED_tf_world_base = ENU_tf_world_base;
NED_tf_world_isaacLidar = NED_tf_world_base;
NED_tf_base_issaacLidar = NED_tf_world_base;
NED_tf_world_base_odom = NED_tf_world_base;


[sensor_pos, sensor_quat] = se3_to_pose(tf_NED_ENU * ...
        ENU_tf_base_mastMount (:,:,1) * ENU_tf_mastMount_sensorMount(:,:,1) * ...
        ENU_tf_sensorMount_lidar(:,:,1) * tf_lidar_isaacLidar * tf_isaacLidar_NED);


dataset.pos_body_sensor = sensor_pos;
dataset.quat_body_sensor = sensor_quat;

draw = 1;
meas_ind = 1;
for ii = 1:size(ENU_tf_world_base,3)
    

    NED_tf_world_base(:,:,ii) = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * tf_ENU_NED;
    [pos, quat] = se3_to_pose(NED_tf_world_base(:,:,ii));
  
    

    NED_tf_world_isaacLidar(:,:,ii) = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar * tf_isaacLidar_NED;

    [sensor_pos, sensor_quat] = se3_to_pose(NED_tf_world_isaacLidar(:,:,ii));


    NED_tf_base_issaacLidar(:,:,ii) = tf_NED_ENU * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar * tf_isaacLidar_NED;

    if dataset.lidar_avail(ii) == 1
        if use_noisy == 1
            cloud_filename = sprintf('cloud_noisy/%d.csv',timing_data(ii,1));
        else
            cloud_filename = sprintf('cloud/%d.csv',timing_data(ii,1));
        end

        cloud_data_lidar = readmatrix(horzcat(path_to_folder,cloud_filename));
        
        cloud_data_lidar = pointCloud(cloud_data_lidar);
        
        % Downsample
        cloud_data_lidar = pcdownsample(cloud_data_lidar,"gridAverage",0.08);
        cloud_data_lidar = cloud_data_lidar.Location';
        
        % Rotate to align with gravity vector
        temp_tf = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar;
        lidar_eul = rotm2eul(temp_tf(1:3,1:3),"ZYX");
        lidar_eul(1) = 0;
        temp_tf = eye(4); temp_tf (1:3,1:3) = eul2rotm(lidar_eul,'ZYX');
        
        NED_cloud_aligned = apply_transform(temp_tf, cloud_data_lidar);

        inverse_temp_tf = inv(temp_tf);
        
        %image_dilation
        [~, peak_keypoints] = detect_peak(NED_cloud_aligned, 0.1, 15);
        %[~, peak_keypoints] = detect_peak(NED_cloud_aligned, 0.2, 15);
        
        % Crater
        [cloud_in_base, crater_keypoints] = detect_crater(NED_cloud_aligned, 60, 10, 0.99,15);
        
        % Rotate keypoint back to sensor frame
        peak_keypoints_sens = apply_transform(inverse_temp_tf, peak_keypoints);
        peak_keypoints_sens = apply_transform(tf_NED_isaacLidar, peak_keypoints_sens);
        crater_keypoints_sens = apply_transform(inverse_temp_tf, crater_keypoints);
        crater_keypoints_sens = apply_transform(tf_NED_isaacLidar, crater_keypoints_sens);
        
        % Generate range-bearing-elevation measurements
        [peak_r, peak_b, peak_e]  = cartesian_to_rbe(peak_keypoints_sens);
        [crater_r, crater_b, crater_e] = cartesian_to_rbe(crater_keypoints_sens);
        
        peak_measurement_vec{meas_ind,1} = [peak_r; peak_b; peak_e];
        crater_measurement_vec{meas_ind,1} = [crater_r; crater_b; crater_e];

        meas_ind = meas_ind + 1;
        reprojected_meas1 = reproject_meas(sensor_pos, sensor_quat, peak_keypoints_sens,sensor_param);
        reprojected_meas2 = reproject_meas(sensor_pos, sensor_quat, crater_keypoints_sens,sensor_param);

        if draw == 1
            figure(1)
            draw_trajectory_se3(eye(4), eye(4), 1, 2, 'k', 0)
            draw_trajectory_se3(NED_tf_world_base(:,:,ii), NED_tf_world_base(:,:,1:ii), 1, 2, 'k', 1)
            draw_trajectory(pos, quat, pos, 0.5, 4,'none',1)
            draw_trajectory(sensor_pos, sensor_quat, sensor_pos, 0.5, 4,'none',1)
            draw_trajectory_se3(NED_tf_world_isaacLidar(:,:,ii), NED_tf_world_isaacLidar(:,:,1:ii), 1, 2, 'none', 1)
            hold on
            scatter3(reprojected_meas1(1,:), reprojected_meas1(2,:), reprojected_meas1(3,:),'b*')
            scatter3(reprojected_meas2(1,:), reprojected_meas2(2,:), reprojected_meas2(3,:),'r*')
            xlabel("X (m)");
            ylabel("Y (m)");
            zlabel("Z (m)");
            axis equal;
            set(gca, 'Zdir', 'reverse')
            set(gca, 'Ydir', 'reverse')
            xlim([-5 50])
            ylim([-15 15])
            zlim([-10 2])
            view([0 90])
            grid on
            drawnow
        end
    end
    
    

end

[pos, quat] = se3_to_pose(NED_tf_world_base);
dataset.pos = pos;
dataset.quat = quat;

[sensor_pos, sensor_quat] = se3_to_pose(NED_tf_world_isaacLidar);
dataset.pos_sensor = sensor_pos;
dataset.quat_sensor = sensor_quat;
dataset.crater_meas_table = crater_measurement_vec;
dataset.peak_meas_table = peak_measurement_vec;