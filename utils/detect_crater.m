function [filtered_cloud, peaks_cloud] = detect_crater(input_cloud, coarse_normal_tol_deg, range_limit)
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
    
    % Find points where normal points toward sensor [0, 0, 0]
    unit_sensor_to_point = normalize(filtered_cloud',2,'norm');
    cos_ang = dot(unit_sensor_to_point, normals, 2);
    cos_tol = cos(deg2rad(coarse_normal_tol_deg));
    normal_check_cloud = filtered_cloud(:,abs(cos_ang) > cos_tol);

    % Cluster
    normalCloud = pointCloud(normal_check_cloud');
    [labels, num_cluster] = pcsegdist(normalCloud,0.5);

    figure()
    pcshow(normalCloud.Location, labels,"MarkerSize",10)
    colormap(hsv(num_cluster))


    x = filterCloud.Location(1:1:end,1);
    y = filterCloud.Location(1:1:end,2);
    z = filterCloud.Location(1:1:end,3);
    u = normals(1:1:end,1);
    v = normals(1:1:end,2);
    w = normals(1:1:end,3);
    
    figure()
    scatter3(filtered_cloud(1,:), filtered_cloud(2,:),filtered_cloud(3,:),1 * ones(size(filtered_cloud)),"blue",'filled')
    hold on
    %quiver3(x,y,z,u,v,w);
    scatter3(normal_check_cloud(1,:), normal_check_cloud(2,:),normal_check_cloud(3,:),20 * ones(size(normal_check_cloud)),"blue",'filled')
    scatter3(0,0,0)
    hold off
    axis equal

end