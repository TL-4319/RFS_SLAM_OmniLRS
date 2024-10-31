close all
clear
clc

%% This script read the recorded bags and convert them to csv type files for processing
dataset_name = 'rosbag2_2024_10_21-09_18_07';

path_to_folder = horzcat('datasets/raw/', dataset_name);
bag = ros2bagreader(path_to_folder);

%% Get imu data and convert to csv
imu_sel = select(bag,"Topic","/imu");

imu_timestamp = imu_sel.MessageList.Time;
imu_msg = readMessages(imu_sel);

% Pre allocate imu data array
% [timestamp_s, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, quat_x, quat_y, quat_z, quat_w]
imu_data = zeros(size(imu_timestamp,1),11);
imu_data(:,1) = imu_timestamp;
for ii = 1:size(imu_data,1)
    imu_data(ii,2:4) = [imu_msg{ii,1}.angular_velocity.x, imu_msg{ii,1}.angular_velocity.y, imu_msg{ii,1}.angular_velocity.z];
    imu_data(ii,5:7) = [imu_msg{ii,1}.linear_acceleration.x, imu_msg{ii,1}.linear_acceleration.y, imu_msg{ii,1}.linear_acceleration.z];
    imu_data(ii,8:11) = [imu_msg{ii,1}.orientation.x, imu_msg{ii,1}.orientation.y, imu_msg{ii,1}.orientation.z, imu_msg{ii,1}.orientation.w];
end


%% Get pose data
tf_sel = select(bag,"Topic","/tf");

tf_timestamp = tf_sel.MessageList.Time;
tf_msg = readMessages(tf_sel);

% Only extract certain transformation include world->base, base->lidar,
% base->imu and base->left_cam
% Note, the world->base will have translational origin of the initial pos
% instead of simulation origin

% All transformation file are as follow
% [timestamp_s, trans_x, trans_y, trans_z, quat_x, quat_y, quat_z, quat_w]

tf_base_world = zeros(size(tf_timestamp,1), 8);
tf_base_world(:,1) = tf_timestamp;
tf_imu_base = tf_base_world;
tf_lcam_base = tf_base_world;
tf_lidar_base = tf_base_world;

% Do mapping of link name to their index in transform tree
for ii = 1:size(tf_msg{1,1}.transforms,1)
    link_name = tf_msg{1,1}.transforms(ii).child_frame_id;
    if strcmp(link_name,'base_link')
        base_ind = ii;
    elseif strcmp(link_name,'imu_link')
        imu_ind = ii;
    elseif strcmp (link_name ,'left_cam_link')
        lcam_ind = ii;
    elseif strcmp(link_name, 'lidar_link')
        lidar_ind = ii;
    end
end

for ii = 1:size(tf_timestamp,1)
    tf_base_world(ii,2:end) = [tf_msg{ii,1}.transforms(base_ind).transform.translation.x, tf_msg{ii,1}.transforms(base_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(base_ind).transform.translation.z, tf_msg{ii,1}.transforms(base_ind).transform.rotation.x, ...
        tf_msg{ii,1}.transforms(base_ind).transform.rotation.y, tf_msg{ii,1}.transforms(base_ind).transform.rotation.z, ...
        tf_msg{ii,1}.transforms(base_ind).transform.rotation.w];

    tf_imu_base(ii,2:end) = [tf_msg{ii,1}.transforms(imu_ind).transform.translation.x, tf_msg{ii,1}.transforms(imu_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(imu_ind).transform.translation.z, tf_msg{ii,1}.transforms(imu_ind).transform.rotation.x, ...
        tf_msg{ii,1}.transforms(imu_ind).transform.rotation.y, tf_msg{ii,1}.transforms(imu_ind).transform.rotation.z, ...
        tf_msg{ii,1}.transforms(imu_ind).transform.rotation.w];

    tf_lcam_base(ii,2:end) = [tf_msg{ii,1}.transforms(lcam_ind).transform.translation.x, tf_msg{ii,1}.transforms(lcam_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(lcam_ind).transform.translation.z, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.x, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.y, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.z, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.w];
        
    tf_lidar_base(ii,2:end) = [tf_msg{ii,1}.transforms(lidar_ind).transform.translation.x, tf_msg{ii,1}.transforms(lidar_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(lidar_ind).transform.translation.z, tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.x, ...
        tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.y, tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.z, ...
        tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.w];
end

% Subtract world->base translation with initial pos 
tf_base_world(:,2:4) = tf_base_world(:,2:4) - tf_base_world(1,2:4);

%% Pre process point cloud
cloud_sel = select(bag,"Topic","/cloud");

cloud_timestamp = cloud_sel.MessageList.Time;
cloud_msg = readMessages(cloud_sel);

cloud_data = cell(size(cloud_timestamp,1),1);

for ii = 1:size(cloud_timestamp,1)
    cur_data = cloud_msg{ii,1}.data;
    point_step = cloud_msg{ii,1}.point_step;
    num_point = cloud_msg{ii,1}.width;
    point_loc = zeros(num_point,3);
    for jj = 0:num_point-1
        % Convert data from byte stream to float32 value

        point_loc(jj+1,1) = typecast([cur_data(jj * point_step + 1), ...
            cur_data(jj * point_step + 2), cur_data(jj * point_step + 3),...
            cur_data(jj * point_step + 4)],"single");

        point_loc(jj+1,2) = typecast([cur_data(jj * point_step + 5), ...
            cur_data(jj * point_step + 6), cur_data(jj * point_step + 7),...
            cur_data(jj * point_step + 8)],"single");

        point_loc(jj+1,3) = typecast([cur_data(jj * point_step + 9), ...
            cur_data(jj * point_step + 10), cur_data(jj * point_step + 11),...
            cur_data(jj * point_step + 12)],"single");
    end
    cloud_data{ii,1} = point_loc;
end

