close all
clc
clear

addpath utils/

%% Get tf of base and lidar
dataset_path = 'datasets/preprocess/test1_bag/';
timing_data = readmatrix(horzcat(dataset_path,'timing.csv'));
dt_s = mean(diff(timing_data(:,1)) * 1e-3);

% TF convention
% TF_B_A = TF_A^B or transformation from A to B or a frame A relative
% to frame B
[tf_timestamps, tf_world_base] = file2se3(horzcat(dataset_path,'base_world_tf.csv'));
[~, tf_base_mastMount] = file2se3(horzcat(dataset_path,'mast_mount_base_tf.csv'));
[~, tf_mastMount_sensorMount] = file2se3(horzcat(dataset_path,'sensor_mount_mast_mount_tf.csv'));
[~, tf_sensorMount_lidar]= file2se3(horzcat(dataset_path,'lidar_sensor_mount_tf.csv'));

% Conversion from blickfeld frame(ENU) to isaac lidar frame (NWU) for
% synthetic data processing
tf_isaacLidar_lidar = eye(4);
tf_isaacLidar_lidar(1:2, 1:2) = [0 -1; 1 0];


v = VideoWriter('video.avi','Motion JPEG AVI');
v.FrameRate = 10;
open(v);


for ii = 1:size(timing_data,1)
    tf_world_isaacLidar = tf_world_base(:,:,ii) * tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;

    tf_base_isaacLidar = tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;
    

    % Check if lidar cloud exist
    if timing_data(ii,2) == 1
        % Read the cloud data
        cloud_filename = sprintf('cloud/%d.csv',timing_data(ii,1));
        cloud_data_lidar = transpose(readmatrix(horzcat(dataset_path,cloud_filename)));

        % Transform the cloud to match robot pose

        
        %cloud_in_base = apply_transform(tf_base_isaacLidar, cloud_data_lidar);
        tf_base_isaacLidar_no_trans = tf_base_isaacLidar;
        tf_base_isaacLidar_no_trans(1:3,4) = [0;0;0];
        cloud_in_base = apply_transform(tf_base_isaacLidar_no_trans, cloud_data_lidar);
        
        % Apply keypoint detector
        %[cloud_in_base, keypoints] = detect_peak(cloud_in_base, 0.1, 7);
        [cloud_in_base, keypoints] = detect_crater(cloud_in_base, 50, 15);

        % Plotting
        hold off
        scatter3(keypoints(1,:), keypoints(2,:),keypoints(3,:),'filled')
        %scatter3(cloud_in_base(1,:),cloud_in_base(2,:), cloud_in_base(3,:),2 * ones(size(cloud_in_base,2),1));
        hold on
        scatter3(cloud_in_base(1,:),cloud_in_base(2,:), cloud_in_base(3,:),2 * ones(size(cloud_in_base,2),1));
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        axis equal
        drawnow
        grid on
        f = getframe(gcf);
        writeVideo(v,f);
    end
end
close(v);
