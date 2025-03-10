close all
clear
clc

%% This script read the recorded bags and convert them to csv type files for processing
dataset_name = 'straight_stereo';

path_to_folder = horzcat('../datasets/raw/opt/', dataset_name);
bag = ros2bagreader(path_to_folder);

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
tf_lcam_sensor_mount = tf_base_world;
tf_rcam_sensor_mount = tf_base_world;
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
    elseif strcmp(link_name, 'right_cam_link')
        rcam_ind = ii;
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

    tf_lcam_sensor_mount(ii,2:end) = [tf_msg{ii,1}.transforms(lcam_ind).transform.translation.x, tf_msg{ii,1}.transforms(lcam_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(lcam_ind).transform.translation.z, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.x, tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(lcam_ind).transform.rotation.z];
        
    tf_rcam_sensor_mount(ii,2:end) = [tf_msg{ii,1}.transforms(rcam_ind).transform.translation.x, tf_msg{ii,1}.transforms(rcam_ind).transform.translation.y,...
        tf_msg{ii,1}.transforms(rcam_ind).transform.translation.z, tf_msg{ii,1}.transforms(rcam_ind).transform.rotation.w, ...
        tf_msg{ii,1}.transforms(rcam_ind).transform.rotation.x, tf_msg{ii,1}.transforms(rcam_ind).transform.rotation.y, ...
        tf_msg{ii,1}.transforms(rcam_ind).transform.rotation.z];

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
tf_lcam_sensor_mount(:,1) = tf_base_world(:,1);
tf_rcam_sensor_mount(:,1) = tf_base_world(:,1);
tf_sensor_mount_mast_mount(:,1) = tf_base_world(:,1);
tf_mast_mount_base(:,1) = tf_base_world(:,1);

% Subtract world->base translation with initial pos 
tf_base_world(:,2:4) = tf_base_world(:,2:4) - tf_base_world(1,2:4);

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

%% Pre process images
img_sel = select(bag,"Topic","/left_image");


img_msg = readMessages(img_sel);

l_img_data = cell(img_sel.NumMessages,1);

l_img_timestamps = zeros(img_sel.NumMessages,1);

for ii = 1:img_sel.NumMessages
    l_img_timestamps(ii) = double(img_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(img_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);

    cur_image = rosReadImage(img_msg{ii,1},"Encoding",img_msg{ii,1}.encoding);
    
    l_img_data{ii,1} = cur_image;
end

%% Pre process images
img_sel = select(bag,"Topic","/right_image");


img_msg = readMessages(img_sel);

r_img_data = cell(img_sel.NumMessages,1);

r_img_timestamps = zeros(img_sel.NumMessages,1);

for ii = 1:img_sel.NumMessages
    r_img_timestamps(ii) = double(img_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(img_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);

    cur_image = rosReadImage(img_msg{ii,1},"Encoding",img_msg{ii,1}.encoding);
    
    r_img_data{ii,1} = cur_image;
end

%% Pre process images
img_sel = select(bag,"Topic","/depth");


img_msg = readMessages(img_sel);

depth_data = cell(img_sel.NumMessages,1);

depth_timestamps = zeros(img_sel.NumMessages,1);

for ii = 1:img_sel.NumMessages
    depth_timestamps(ii) = double(img_msg{ii,1}.header.stamp.sec(1) * 1e3) + double(img_msg{ii,1}.header.stamp.nanosec(1) * 1e-6);

    cur_image = rosReadImage(img_msg{ii,1},"Encoding",img_msg{ii,1}.encoding);
    
    depth_data{ii,1} = cur_image;
end

%% Write datas from ROS bag to cvs types
% Output dir structure
%   datasets/preprocess/<dataset_name>/
%       |_  timings.csv
%       |_  imu.csv
%       |_  odom.csv
%       |_  base_world_tf.csv
%       |_  imu_base_tf.csv
%       |_  lcam_base_tf.csv ...
%       |_  l_image/
%       |       |_ <timestamp>.png
%       |       |_      ...       
%       |_  r_image/
%               |_ <timestamp>.png
%               |_      ...    
pre_process_output_dir = horzcat('../datasets/preprocess/opt/', dataset_name);
[~,~] = rmdir(pre_process_output_dir,'s');
mkdir(pre_process_output_dir)

%% Timing file
%  timing.csv
%       |_  timestamp_ms, lidar_avail, img_avail
timing_data = zeros(size(imu_data,1),3);
timing_data(:,1) = imu_data(:,1);
%timing_data(:,2) = ismember(imu_data(:,1),cloud_timestamps);
timing_data(:,3) = ismember(imu_data(:,1),l_img_timestamps);

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

lcam_base_filename = horzcat(pre_process_output_dir,'/lcam_base_tf.csv');
writematrix(tf_lcam_sensor_mount, lcam_base_filename)

rcam_base_filename = horzcat(pre_process_output_dir,'/rcam_base_tf.csv');
writematrix(tf_rcam_sensor_mount, lcam_base_filename)


%% Image data
%   l_image/<timestamp_ms>.png

l_img_path = horzcat(pre_process_output_dir, '/l_image');
r_img_path = horzcat(pre_process_output_dir, '/r_image');
depth_path = horzcat(pre_process_output_dir, '/depth');
mkdir(l_img_path)
mkdir(r_img_path)
mkdir(depth_path)
parfor ii = 1:size(l_img_data,1)
    img_name = sprintf("%s/%d.png",l_img_path, round(l_img_timestamps(ii)));
    imwrite(l_img_data{ii,1}, img_name)
    img_name = sprintf("%s/%d.png",r_img_path, round(l_img_timestamps(ii)));
    imwrite(r_img_data{ii,1}, img_name)
    img_name = sprintf("%s/%d.csv",depth_path, round(l_img_timestamps(ii)));
    writematrix(depth_data{ii,1}, img_name)
end

% avg_framerate = 1/mean(diff(img_timestamps) * 1e-3);
% video_filename = horzcat(pre_process_output_dir, '/video.avi');
% v = VideoWriter(video_filename,'Motion JPEG AVI');
% v.FrameRate = avg_framerate;
% open(v);
% for ii = 1:size(img_data,1)
%     writeVideo(v,img_data{ii,1});
% end
% close(v);
