close all
clc
clear

addpath ../utils/

coarse_ang_array =10:5:85;
min_num_clust = 10;
ray_trace_step = 0.99;
max_range = 15;

for kk = 1:size(coarse_ang_array,2)
coarse_ang = coarse_ang_array(kk);

detector_str = sprintf("detect_crater(cloud_in_base, %d, %d, %.2f,%d)",coarse_ang, min_num_clust, ray_trace_step, max_range);
v_name = strcat("sts_noisy_",detector_str,".avi");
f_name = strcat("sts_noisy_",detector_str,".mat");

%% Get tf of base and lidar
dataset_path = 'datasets/preprocess/straight_turn_straight/';
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



 v = VideoWriter(v_name,'Motion JPEG AVI');
 v.FrameRate = 10;
 open(v);

 f = cell(size(timing_data,1),1);

 num_meas = sum(timing_data(:,2));

 meas_cell = cell(num_meas,1);
 meas_ind = 1;

for ii = 1:size(timing_data,1)

    tf_world_isaacLidar = tf_world_base(:,:,ii) * tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;

    tf_base_isaacLidar = tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;
    

    % Check if lidar cloud exist
    if timing_data(ii,2) == 1
        % Read the cloud data
        cloud_filename = sprintf('cloud_noisy/%d.csv',timing_data(ii,1));
        cloud_data_lidar = readmatrix(horzcat(dataset_path,cloud_filename));

        cloud_data_lidar = pointCloud(cloud_data_lidar);
        
        % Downsample
        cloud_data_lidar = pcdownsample(cloud_data_lidar,"gridAverage",0.08);
        cloud_data_lidar = cloud_data_lidar.Location';
        % Transform the cloud to match robot pose

        
        %cloud_in_base = apply_transform(tf_base_isaacLidar, cloud_data_lidar);
        tf_base_isaacLidar_no_trans = tf_base_isaacLidar;
        tf_base_isaacLidar_no_trans(1:3,4) = [0;0;0];
        cloud_in_base = apply_transform(tf_base_isaacLidar_no_trans, cloud_data_lidar);
        
        % Apply keypoint detector
        %[cloud_in_base, keypoints] = detect_peak(cloud_in_base, 0.1, 15);
        [cloud_in_base, keypoints] = detect_crater(cloud_in_base, coarse_ang, min_num_clust, ray_trace_step,max_range);
        
        meas_cell{meas_ind,1} = keypoints;
        meas_ind = meas_ind+1;

        % Plotting
        figure(1)
        %figure(1,"Position",[1,1,700,400])
        hold off
        cloudInBase = pointCloud(cloud_in_base',"Color",'white');
        
        pcshow(cloudInBase,"MarkerSize",10,'AxesVisibility','off')
        if size(keypoints,2) > 0
            Keypoints = pointCloud(keypoints','Color','red');
            hold on
            pcshow(Keypoints,'MarkerSize',100,'AxesVisibility','off')
        end 
        xlabel('X (m)')
        ylabel('Y (m)')
        zlabel('Z (m)')
        
        
        
        grid on
        view([0 40])
        axis equal
        ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','k');
        ylim([2 15])
        xlim([-9 9])
        zlim([-3 0])
        drawnow
        
        f = getframe(gcf);
        
        f.cdata = insertText(f.cdata, [10 10],detector_str);
        writeVideo(v,f);
    end
end
close(v);

save(f_name, "meas_cell","-v7.3")

end