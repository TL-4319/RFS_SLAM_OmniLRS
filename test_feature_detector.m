close all
clc
clear

addpath utils/

%% Get cloud
cloud_path = 'datasets/preprocess/test1/cloud/';

name = dir(cloud_path);

cloud_timestamp = zeros(size(name,1)-2,1);

cloud = cell(size(name,1)-2,1);

for ii = 3:size(name,1)
    cloud_name = name(ii).name;
    
    cloud_timestamp(ii-2,1) = str2double(strrep(cloud_name,'.csv',''));

    cloud_name = horzcat(cloud_path, cloud_name);
    cloud_data = readmatrix(cloud_name);
    
    cloud{ii-2,1} = cloud_data;
end

%% Get tf of base and lidar
tf_path = 'datasets/preprocess/test1/';

[tf_timestamps, base_world_tf] = file2se3(horzcat(tf_path,'base_world_tf.csv'));
[~, mast_mount_base_tf] = file2se3(horzcat(tf_path,'mast_mount_base_tf.csv'));
[~, sensor_mount_mast_mount_tf] = file2se3(horzcat(tf_path,'sensor_mount_mast_mount_tf.csv'));
[~, lidar_sensor_mount_tf] = file2se3(horzcat(tf_path,'lidar_sensor_mount_tf.csv'));

lidar_world_tf = base_world_tf .* mast_mount_base_tf .*...
    sensor_mount_mast_mount_tf .* lidar_sensor_mount_tf;

for ii = 1:size(tf_timestamps)
    draw_trajectory(base_world_tf(ii), [0; 0; 0], 3, 2, 'k', false)
    draw_trajectory(lidar_world_tf(ii),[0; 0; 0], 3, 2, 'k', true)  
    axis equal
    ylim([-20 20])
    xlim([-20 20])
    zlim([-20 20])
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    
    drawnow
end