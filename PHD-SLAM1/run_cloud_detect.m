function rbe = run_cloud_detect(dataset_name, cloud_name, sensor_params, pose_ind)
    addpath '..'

    %% Read pose and odom
    [~, ENU_tf_world_base] = file2se3(strcat(dataset_name,'base_world_tf.csv')); % ENU
    [~, ENU_tf_base_mastMount] = file2se3(horzcat(dataset_name,'mast_mount_base_tf.csv'));
    [~, ENU_tf_mastMount_sensorMount] = file2se3(horzcat(dataset_name,'sensor_mount_mast_mount_tf.csv'));
    [~, ENU_tf_sensorMount_lidar]= file2se3(horzcat(dataset_name,'lidar_sensor_mount_tf.csv'));

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

    %% Read cloud
    cloud_name = strcat(dataset_name,"/cloud/",cloud_name);
    cloud_data = readmatrix(cloud_name);

    cloud_data = add_pc_noise(cloud_data, sensor_params.lidar_noise);

    % Rotate to align with gravity vector
    temp_tf = tf_NED_ENU * ENU_tf_world_base(:,:,pose_ind) * ...
        ENU_tf_base_mastMount (:,:,pose_ind) * ENU_tf_mastMount_sensorMount(:,:,pose_ind) * ...
        ENU_tf_sensorMount_lidar(:,:,pose_ind) * tf_lidar_isaacLidar;
    lidar_eul = rotm2eul(temp_tf(1:3,1:3),"ZYX");
    lidar_eul(1) = 0;
    temp_tf = eye(4); temp_tf (1:3,1:3) = eul2rotm(lidar_eul,'ZYX');
    inv_temp_tf = inv(temp_tf);
        
    cloud_data = apply_transform(temp_tf, cloud_data');
    cloud_data = pointCloud(cloud_data');

    % Downsample
    cloud_data = pcdownsample(cloud_data,"gridAverage",0.08);
    cloud_data = cloud_data.Location';
    
    %Run detector
    if sensor_params.detector == "peak"
        [~, keypoints] = detect_peak(cloud_data, 0.15, 30);
    elseif sensor_params.detector == "crater"
        [~, keypoints] = detect_crater(cloud_data, 70, 10, 0.99,30);
    end
    
    num_keypoints = size(keypoints,2);
    if num_keypoints > 0
        keypoints = apply_transform(inv_temp_tf, keypoints);
        keypoints = apply_transform(tf_NED_isaacLidar,keypoints);
    
        rbe = zeros(3,num_keypoints);
        [rbe(1,:), rbe(2,:), rbe(3,:)]  = cartesian_to_rbe(keypoints);
    else
        rbe = [];
    end
    
end