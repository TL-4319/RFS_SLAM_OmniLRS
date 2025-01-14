function [filtered_cloud, peaks_cloud] = detect_crater(input_cloud, coarse_normal_tol_deg, ...
    min_point_per_cluster, ray_trace_step,range_limit)
    if range_limit == 0
        filtered_cloud = input_cloud;
    else
        % Filter cloud based on range
        dist_from_sensor = (input_cloud(1,:).^2 + input_cloud(2,:).^2).^0.5;
        filtered_cloud = input_cloud(:,find(dist_from_sensor< range_limit));
    end
    
    % Find normal
    filterCloud = pointCloud(filtered_cloud');
    normals = pcnormals(filterCloud,15);
    
    % Find points where normal points toward sensor at [0, 0, 0]
    unit_sensor_to_point = normalize(filtered_cloud',2,'norm');
    cos_ang = dot(unit_sensor_to_point, normals, 2);
    cos_tol = cos(deg2rad(coarse_normal_tol_deg));
    normal_check_cloud = filtered_cloud(:,abs(cos_ang) > cos_tol);

    % Cluster
    normalCloud = pointCloud(normal_check_cloud');
    [labels, num_cluster] = pcsegdist(normalCloud,0.2,"NumClusterPoints",min_point_per_cluster);

    % figure(1)
    % pcshow(normalCloud.Location, labels,"MarkerSize",10)
    % colormap(hsv(num_cluster))
    
    % Crater detectiom for each back wall cluster
    filtered_cloud_XY = filtered_cloud(1:2,:);
    peaks_cloud = [];
    for ii = 1:num_cluster
        % Detect front of crater using ray casting
        current_cluster = normal_check_cloud(:,labels == ii); % Get back wall cluster
        current_cluster_normal = normals(labels == ii,:); % Get normal
        back_centroid = mean(current_cluster,2);
        dist_to_back_centroid = norm(back_centroid);
        unit_to_centroid = normalize(back_centroid',2,'norm');
        dist_sample = ray_trace_step * dist_to_back_centroid;
        prev_gap = 0;
        iter = 1;
        % Ray cast from backwall toward the sensor
        while iter < 1000
            % For a vertical ray to intersect with data, there exist
            % pointcloud within XY distance thresshold from current sample
            % point
            sample_along_ray = unit_to_centroid * dist_sample;
            sample_XY = sample_along_ray(1:2);
            XY_dist_squared = filtered_cloud_XY' - sample_XY;
            dist_squared = XY_dist_squared(:,1).^2 + XY_dist_squared(:,2).^2;
            num_point_intersect = sum(dist_squared < 0.05^2);
            
            % Quick why to reject false detection due to pointcloud
            % sparesity
            if iter < 4 && num_point_intersect == 0
                break
            end

            % When newly encouter data gap
            if  prev_gap == 0 && num_point_intersect == 0
                prev_gap = 1;
            end

            % When encouter data after a gap. Front crater rim
            if prev_gap == 1 && num_point_intersect > 0
                [~, min_dist_to_ray_ind] = min(dist_squared);
                crater_front = filtered_cloud(:,min_dist_to_ray_ind);
                current_rough_center = (crater_front + back_centroid)./2 ;
                
                % Crater center refinement
                % Normal refinement only need 2D normal in XY now
                current_cluster_normal = current_cluster_normal';
                current_cluster_normal(3,:) = 0;
                current_cluster_normal = normalize(current_cluster_normal,1,"norm");
                
                current_cluster_to_center = current_cluster - current_rough_center;
                current_cluster_to_center(3,:) = 0;
                current_cluster_to_center = normalize(current_cluster_to_center,1,"norm");
                cos_angle_cluster = current_cluster_normal .* current_cluster_to_center;
                cos_angle_cluster = sum(cos_angle_cluster,1);
                cos_tol_refine = cos(deg2rad(30));
                refine_cluster = current_cluster(:,cos_angle_cluster>cos_tol_refine);
                refine_dist = (refine_cluster(1,:).^2 + refine_cluster(2,:).^2).^0.5;
                [~,far_ind] = max(refine_dist);
                crater_back = refine_cluster(:,far_ind);
                refine_center = (crater_back + crater_front)./2;
                
                % Output
                peaks_cloud = horzcat(peaks_cloud, current_rough_center);
                break
            end
            dist_sample = dist_sample * ray_trace_step;
            iter = iter + 1;
        end      

    end

    % x = filterCloud.Location(1:1:end,1);
    % y = filterCloud.Location(1:1:end,2);
    % z = filterCloud.Location(1:1:end,3);
    % u = normals(1:1:end,1);
    % v = normals(1:1:end,2);
    % w = normals(1:1:end,3);
    % 
    % figure()
    % scatter3(filtered_cloud(1,:), filtered_cloud(2,:),filtered_cloud(3,:),1 * ones(size(filtered_cloud)),"blue",'filled')
    % hold on
    % %quiver3(x,y,z,u,v,w);
    % scatter3(normal_check_cloud(1,:), normal_check_cloud(2,:),normal_check_cloud(3,:),20 * ones(size(normal_check_cloud)),"blue",'filled')
    % scatter3(crater_front(1,:), crater_front(2,:),crater_front(3,:),20 * ones(size(crater_front)),"red",'filled')
    % scatter3(0,0,0)
    % hold off
    % axis equal

end