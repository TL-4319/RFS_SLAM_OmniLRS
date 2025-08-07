close all
clear
clc

addpath utils/

lidar_noise.range_m = 0.1;
lidar_noise.angular_deg = 0.;

%% This script read the recorded bags and convert them to csv type files for processing
dataset_name = 'rosbag2_2025_08_06-11_42_01';
%dataset_name = 'straight_turn_straight';

path_to_folder = horzcat('datasets/preprocess/lidar/', dataset_name,'/');
load("blickfeld_cube1_400_scan_pattern.mat");

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

%% Read timing information
timing_data = readmatrix(strcat(path_to_folder,'timing.csv'));

for ii = 1:size(timing_data,1)
    if timing_data(ii,2) == 0
        continue
    end

    % Frame transformation
    NED_tf_world_base = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * tf_ENU_NED;
  
    NED_tf_world_isaacLidar = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar * tf_isaacLidar_NED;


    NED_tf_base_issaacLidar = tf_NED_ENU * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar * tf_isaacLidar_NED;
    
    % Read cloud
    cloud_filename = sprintf('cloud/%d.csv',timing_data(ii,1));

    cloud_data_lidar = readmatrix(horzcat(path_to_folder,cloud_filename));

    cloud_data_lidar = add_pc_noise(cloud_data_lidar, lidar_noise);

    % Rotate to align with gravity vector
        temp_tf = tf_NED_ENU * ENU_tf_world_base(:,:,ii) * ...
        ENU_tf_base_mastMount (:,:,ii) * ENU_tf_mastMount_sensorMount(:,:,ii) * ...
        ENU_tf_sensorMount_lidar(:,:,ii) * tf_lidar_isaacLidar;
        lidar_eul = rotm2eul(temp_tf(1:3,1:3),"ZYX");
        lidar_eul(1) = 0;
        temp_tf = eye(4); temp_tf (1:3,1:3) = eul2rotm(lidar_eul,'ZYX');
        
        NED_cloud_aligned = apply_transform(temp_tf, cloud_data_lidar');
        
    cloud_data_lidar = pointCloud(NED_cloud_aligned');

    % Downsample
    cloud_data_lidar = pcdownsample(cloud_data_lidar,"gridAverage",0.4);

    %points = detectISSFeatures(cloud_data_lidar,NonMaxRadius=1);
    points = detectLOAMFeatures(cloud_data_lidar);
    %[~,points] = detect_peak(NED_cloud_aligned, 0.15, 30); points = points';
    %[~,points] = detect_crater(NED_cloud_aligned, 75,10,0.99,30); points = points';

    pcshow(cloud_data_lidar,"MarkerSize",10)
    hold on
    plot3(points(:,1),points(:,2),points(:,3),"pentagram", MarkerSize=10,MarkerFaceColor=[1 0.6 0.6],Color="red")
    hold off
    xlabel("X")
    ylabel("Y")
    zlabel("Z")
    drawnow


end