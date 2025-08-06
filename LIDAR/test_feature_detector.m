close all
clc
clear

addpath ../utils/

detector_str = "detect_crater(cloud_in_base, 70, 10, 0.99,15)";
v_name = strcat("sts_noisy_",detector_str,".avi");

%% Get tf of base and lidar
dataset_path = 'datasets/preprocess/20250324_omnilrs/';
timing_data = readmatrix(horzcat(dataset_path,'timing.csv'));
dt_s = mean(diff(timing_data(:,1)) * 1e-3);

%init_pos = [32.3711; 35.6074; 1.8]; % Straight
init_pos = [0;0;0];
%init_pos = [32.3711;   35.1074;    0.2855]; % Straight-turn-striaght


% TF convention
% TF_B_A = TF_A^B or transformation from A to B or a frame A relative
% to frame B
[tf_timestamps, tf_world_base] = file2se3(horzcat(dataset_path,'base_world_tf.csv'));
%tf_world_base(3,4,:) = -0.25;
[~, tf_base_mastMount] = file2se3(horzcat(dataset_path,'mast_mount_base_tf.csv'));
[~, tf_mastMount_sensorMount] = file2se3(horzcat(dataset_path,'sensor_mount_mast_mount_tf.csv'));
[~, tf_sensorMount_lidar]= file2se3(horzcat(dataset_path,'lidar_sensor_mount_tf.csv'));

% Conversion from blickfeld frame(ENU) to isaac lidar frame (NWU) for
% synthetic data processing
tf_isaacLidar_lidar = eye(4);
tf_isaacLidar_lidar(1:2, 1:2) = [0 -1; 1 0];



 % v = VideoWriter(v_name,'Motion JPEG AVI');
 % v.FrameRate = 10;
 % open(v);

 f = cell(size(timing_data,1),1);
    
 truth_filename = "all_rock.csv";
 rot_world = eul2rotm(deg2rad([0,-3,2]));
 rot_world = eye(3);

 
 az = 0;
 el = 40;
 z = 1;
for ii = 1:size(timing_data,1)
    tf_world_isaacLidar = tf_world_base(:,:,ii) * tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;

    tf_base_isaacLidar = tf_base_mastMount(:,:,ii) * ...
        tf_mastMount_sensorMount(:,:,ii) * tf_sensorMount_lidar(:,:,ii) * tf_isaacLidar_lidar;
    

    % Check if lidar cloud exist
    if timing_data(ii,2) == 1
        
        truth_peak = readmatrix(truth_filename) - init_pos;
        %truth_peak = rot_world * truth_peak;
        % Read the cloud data
        cloud_filename = sprintf('cloud/%d.csv',timing_data(ii,1));
        cloud_data_lidar = readmatrix(horzcat(dataset_path,cloud_filename));

        cloud_data_lidar = pointCloud(cloud_data_lidar);
        
        % Downsample
        %cloud_data_lidar = pcdownsample(cloud_data_lidar,"gridAverage",0.08);
        cloud_data_lidar = cloud_data_lidar.Location';
        % Transform the cloud to match robot pose

        
        cloud_in_base = apply_transform(tf_base_isaacLidar, cloud_data_lidar);
        tf_base_isaacLidar_no_trans = tf_base_isaacLidar;
        tf_base_isaacLidar_no_trans(1:3,4) = [0;0;0];
        %cloud_in_base = apply_transform(tf_world_isaacLidar, cloud_data_lidar);
        truth_in_base = apply_transform(inv(tf_world_base(:,:,ii)), truth_peak);
        
        % Apply keypoint detector
        %[cloud_in_base, keypoints] = detect_peak(cloud_in_base, 0.1, 15);
        %[cloud_in_base, keypoints] = detect_crater(cloud_in_base, 70, 10, 0.99,15);

        % Plotting
        figure(1)
        hold off
        p = scatter3(cloud_in_base(1,:), cloud_in_base(2,:), cloud_in_base(3,:),'.');
        hold on
        scatter3(truth_in_base(1,:), truth_in_base(2,:), truth_in_base(3,:),50,'filled')
        %scatter3(truth_in_base(1,:), truth_in_base(2,:), truth_in_base(3,:),50,'filled')
        %pcshow(Keypoints,'MarkerSize',100,'AxesVisibility','off')
        %pcshow(cloudTruth,'MarkerSize',500,'AxesVisibility','off')
        xlabel('X (m)')
        ylabel('Y (m)')
        zlabel('Z (m)')
        
        
        
        
        %set(gca,'color','k');
        view([az el])
        %zoom(z)
        axis("equal")
        ylim([2 15])
        xlim([-9 9])
        zlim([-3 3])
        drawnow
        
        %p.ButtonDownFcn = {@click, truth_filename, tf_world_base(:,:,ii)};
        sf = getframe(gcf);
        [az, el] = view();
        %z=zoom();
        
        %f.cdata = insertText(f.cdata, [10 10],detector_str);
        %writeVideo(v,f);
    end
end
%close(v);

function click(obj,event, truth_filename, tf_world_base)
    %fprintf('x = %f, y = %f, z = %f\n',event.IntersectionPoint(1:3))
    
    % Search and remove previous 'recent selection'
    delete(findall(obj.Parent, 'Tag','recentSelection'))

    % display the selected point on plot
    scatter3(event.IntersectionPoint(1), event.IntersectionPoint(2), event.IntersectionPoint(3),...
        200,'rx', 'tag','recentSelection')
    
    save_or_nah = waitforbuttonpress;
    if save_or_nah
        p = get(gcf, 'CurrentCharacter');
        if p == 'y'
            sel_point = [event.IntersectionPoint(1);event.IntersectionPoint(2);event.IntersectionPoint(3)];
            point_in_global = apply_transform(tf_world_base,sel_point);
            truth_peak = readmatrix(truth_filename);
            truth_peak = horzcat(truth_peak,point_in_global);
            writematrix(truth_peak,truth_filename);
        else
            disp("select again")
        end

    end
    
end