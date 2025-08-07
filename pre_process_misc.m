close all
clear
clc

%% This script read the recorded bags and convert them to csv type files for processing
dataset_name = '20250324_omnilrs';

path_to_folder = horzcat('../datasets/raw/lidar/', dataset_name);
bag = ros2bagreader(path_to_folder);

% Standard deviation
lidar_noise.angular_deg = 0.0;
lidar_noise.range_m = 0.05;

%% Get imu data and convert to csv
imu_sel = select(bag,"Topic","/imu");
imu_msg = readMessages(imu_sel);

% Pre allocate imu data array
% [timestamp_ms, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, quat_w, quat_z, quat_y, quat_z]
imu_data = zeros(imu_sel.NumMessages,11);

for ii = 1:size(imu_data,1)
    imu_data(ii,1) = double(imu_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(imu_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);
    imu_data(ii,2:4) = [imu_msg{ii,1}.angular_velocity.x, imu_msg{ii,1}.angular_velocity.y, imu_msg{ii,1}.angular_velocity.z];
    imu_data(ii,5:7) = [imu_msg{ii,1}.linear_acceleration.x, imu_msg{ii,1}.linear_acceleration.y, imu_msg{ii,1}.linear_acceleration.z];
    imu_data(ii,8:11) = [imu_msg{ii,1}.orientation.w, imu_msg{ii,1}.orientation.x, imu_msg{ii,1}.orientation.y, imu_msg{ii,1}.orientation.z];
end


%% Get pose data
tf_sel = select(bag,"Topic","/tf");


tf_msg = readMessages(tf_sel);

% Only extract certain transformation include world->base, base->mast_mount,
% mast_mount->sensor_mount, sensor_mount->lidar, base->imu and base->left_cam
% Note, the world->base will have translational origin of the initial pos
% instead of simulation origin

% All transformation file are as follow
% [timestamp_ms, trans_x, trans_y, trans_z, quat_w, quat_x, quat_y, quat_z]

tf_base_world = zeros(tf_sel.NumMessages, 8);
tf_imu_base = tf_base_world;
tf_lcam_base = tf_base_world;
tf_lidar_sensor_mount = tf_base_world;
tf_sensor_mount_mast_mount = tf_base_world;
tf_mast_mount_base = tf_base_world;

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
    elseif strcmp(link_name, 'mast_mount_link')
        mast_mount_ind = ii;
    elseif strcmp(link_name, 'sensor_mount_link')
        sensor_mount_ind = ii;
    end
end

for ii = 1:tf_sel.NumMessages
    tf_base_world(ii,1) = double(tf_msg{ii,1}.transforms(1).header.stamp.sec(1) * 1e3) + double(tf_msg{ii,1}.transforms(1).header.stamp.nanosec(1)) * 1e-6;

    tf_base_world(ii,2:end) = [tf_msg{ii,1}.transforms(base_ind).transform.translation.x, tf_msg{ii,1}.transforms(base_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(base_ind).transform.translation.z, tf_msg{ii,1}.transforms(base_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(base_ind).transform.rotation.x, tf_msg{ii,1}.transforms(base_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(base_ind).transform.rotation.z];

    tf_imu_base(ii,2:end) = [tf_msg{ii,1}.transforms(imu_ind).transform.translation.x, tf_msg{ii,1}.transforms(imu_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(imu_ind).transform.translation.z, tf_msg{ii,1}.transforms(imu_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(imu_ind).transform.rotation.x, tf_msg{ii,1}.transforms(imu_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(imu_ind).transform.rotation.z];

    tf_lcam_base(ii,2:end) = [tf_msg{ii,1}.transforms(lcam_ind).transform.translation.x, tf_msg{ii,1}.transforms(lcam_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(lcam_ind).transform.translation.z, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.x, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.z];
        
    tf_lidar_sensor_mount(ii,2:end) = [tf_msg{ii,1}.transforms(lidar_ind).transform.translation.x, tf_msg{ii,1}.transforms(lidar_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(lidar_ind).transform.translation.z, tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.x, tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(lidar_ind).transform.rotation.z];

    tf_mast_mount_base(ii,2:end) = [tf_msg{ii,1}.transforms(mast_mount_ind).transform.translation.x, tf_msg{ii,1}.transforms(mast_mount_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(mast_mount_ind).transform.translation.z, tf_msg{ii,1}.transforms(mast_mount_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(mast_mount_ind).transform.rotation.x, tf_msg{ii,1}.transforms(mast_mount_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(mast_mount_ind).transform.rotation.z];

    tf_sensor_mount_mast_mount(ii,2:end) = [tf_msg{ii,1}.transforms(sensor_mount_ind).transform.translation.x, tf_msg{ii,1}.transforms(sensor_mount_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(sensor_mount_ind).transform.translation.z, tf_msg{ii,1}.transforms(sensor_mount_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(sensor_mount_ind).transform.rotation.x, tf_msg{ii,1}.transforms(sensor_mount_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(sensor_mount_ind).transform.rotation.z];
end
tf_imu_base(:,1) = tf_base_world(:,1);
tf_lcam_base(:,1) = tf_base_world(:,1);
tf_lidar_sensor_mount(:,1) = tf_base_world(:,1);
tf_sensor_mount_mast_mount(:,1) = tf_base_world(:,1);
tf_mast_mount_base(:,1) = tf_base_world(:,1);

% Subtract world->base translation with initial pos 
%tf_base_world(:,2:4) = tf_base_world(:,2:4) - tf_base_world(1,2:4);

%% Pre process odom messages
odom_sel = select(bag,"Topic","/odom");
odom_msg = readMessages(odom_sel);

% Pre allocate imu data array
% [timestamp_ms, ang_x, ang_y, ang_z, linear_x, linear_y, linear_z]
odom_data = zeros(odom_sel.NumMessages,7);

for ii = 1:size(odom_data,1)
    odom_data(ii,1) = double(odom_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(odom_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);
    odom_data(ii,2:4) = [odom_msg{ii,1}.twist.twist.angular.x, odom_msg{ii,1}.twist.twist.angular.y odom_msg{ii,1}.twist.twist.angular.z];
    odom_data(ii,5:7) = [odom_msg{ii,1}.twist.twist.linear.x, odom_msg{ii,1}.twist.twist.linear.y, odom_msg{ii,1}.twist.twist.linear.z];
end

%% Pre process point cloud
cloud_sel = select(bag,"Topic","/cloud");

cloud_msg = readMessages(cloud_sel);

cloud_data = cell(cloud_sel.NumMessages,1);

noisy_cloud_data = cloud_data;

cloud_timestamps = zeros(cloud_sel.NumMessages,1);

for ii = 1:cloud_sel.NumMessages
    cloud_timestamps(ii) = double(cloud_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(cloud_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);
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
    % Add noise
    point_loc_noisy = add_pc_noise(point_loc, lidar_noise);

    cloud_data{ii,1} = point_loc;

    noisy_cloud_data{ii,1} = point_loc_noisy;
    
end

%% Pre process images
img_sel = select(bag,"Topic","/left_image");


img_msg = readMessages(img_sel);

img_data = cell(img_sel.NumMessages,1);

img_timestamps = zeros(img_sel.NumMessages,1);

for ii = 1:img_sel.NumMessages
    img_timestamps(ii) = double(img_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(img_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);

    cur_image = rosReadImage(img_msg{ii,1},"Encoding",img_msg{ii,1}.encoding);
    
    img_data{ii,1} = cur_image;
end

%% Write datas from ROS bag to cvs types
% Output dir structure
%   datasets/preprocess/<dataset_name>/
%       |_  timings.csv
%       |_  imu.csv
%       |_  odom.csv
%       |_  base_world_tf.csv
%       |_  lidar_base_tf.csv
%       |_  imu_base_tf.csv
%       |_  lcam_base_tf.csv
%       |_  cloud/
%       |       |_ <timestamp>.csv
%       |       |_      ...
%       |_  image/
%               |_ <timestamp>.png
%               |_      ...       
pre_process_output_dir = horzcat('datasets/preprocess/', dataset_name);
[~,~] = rmdir(pre_process_output_dir,'s');
mkdir(pre_process_output_dir)

%% Timing file
%  timing.csv
%       |_  timestamp_ms, lidar_avail, img_avail
timing_data = zeros(size(imu_data,1),3);
timing_data(:,1) = imu_data(:,1);
timing_data(:,2) = ismember(imu_data(:,1),cloud_timestamps);
timing_data(:,3) = ismember(imu_data(:,1),img_timestamps);

timing_filename = horzcat(pre_process_output_dir, '/timing.csv');
writematrix(timing_data, timing_filename)

%% IMU file
%  imu.csv
%       |_  timestamp_ms, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, quat_x, quat_y, quat_z, quat_w
imu_filename = horzcat(pre_process_output_dir, '/imu.csv');
writematrix(imu_data,imu_filename)

%% Odom file
%  odom.csv
%       |_  timestamp_ms, ang_x, ang_y, ang_z, linear_x, linear_y, linear_z
odom_filename = horzcat(pre_process_output_dir, '/odom.csv');
writematrix(odom_data,odom_filename)

%% TF files
%   <child_parent>_tf.csv
%       |_ timestamp_ms, trans_x, trans_y, trans_z, quat_x, quat_y, quat_z, quat_w

world_base_filename = horzcat(pre_process_output_dir, '/base_world_tf.csv');
writematrix(tf_base_world, world_base_filename)

imu_base_filename = horzcat(pre_process_output_dir, '/imu_base_tf.csv');
writematrix(tf_imu_base, imu_base_filename)

mast_mount_base_filename = horzcat(pre_process_output_dir,'/mast_mount_base_tf.csv');
writematrix(tf_mast_mount_base, mast_mount_base_filename)

sensor_mount_mast_mount_filename = horzcat(pre_process_output_dir,'/sensor_mount_mast_mount_tf.csv');
writematrix(tf_sensor_mount_mast_mount, sensor_mount_mast_mount_filename)

lidar_sensor_mount_filename = horzcat(pre_process_output_dir, '/lidar_sensor_mount_tf.csv');
writematrix(tf_lidar_sensor_mount, lidar_sensor_mount_filename)

lcam_base_filename = horzcat(pre_process_output_dir,'/lcam_base_tf.csv');
writematrix(tf_lcam_base, lcam_base_filename)

%% Cloud data
%   cloud/<timestamp_ms>.csv
%       |_ x, y, z
%       |_  ...

cloud_path = horzcat(pre_process_output_dir, '/cloud');
mkdir(cloud_path)

for ii = 1:size(cloud_data,1)
    cloud_name = sprintf("%s/%d.csv",cloud_path, round(cloud_timestamps(ii)));
    writematrix(cloud_data{ii,1}, cloud_name)
end

noisy_cloud_path = horzcat(pre_process_output_dir, '/cloud_noisy');
mkdir(noisy_cloud_path)

for ii = 1:size(cloud_data,1)
    cloud_name = sprintf("%s/%d.csv",noisy_cloud_path, round(cloud_timestamps(ii)));
    writematrix(noisy_cloud_data{ii,1}, cloud_name)
end

%% Image data
%   image/<timestamp_ms>.png

img_path = horzcat(pre_process_output_dir, '/image');
mkdir(img_path)

parfor ii = 1:size(img_data,1)
    img_name = sprintf("%s/%d.png",img_path, round(img_timestamps(ii)));
    imwrite(img_data{ii,1}, img_name)
end

avg_framerate = 1/mean(diff(img_timestamps) * 1e-3);
video_filename = horzcat(pre_process_output_dir, '/video.avi');
v = VideoWriter(video_filename,'Motion JPEG AVI');
v.FrameRate = avg_framerate;
open(v);
for ii = 1:size(img_data,1)
    writeVideo(v,img_data{ii,1});
end
close(v);
