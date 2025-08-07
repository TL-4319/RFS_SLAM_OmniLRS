function [filtered_cloud, peaks_cloud] = detect_peak(input_cloud, res_scale, range_limit)
    
    if range_limit == 0
        filtered_cloud = input_cloud;
    else
        % Filter cloud based on range
        dist_from_sensor = (input_cloud(1,:).^2 + input_cloud(2,:).^2).^0.5;
        filtered_cloud = input_cloud(:,find(dist_from_sensor< range_limit));
    end
    

    is_peak = zeros(1,size(filtered_cloud,2));

    for ii = 1:size(filtered_cloud,2)
        cur_point = filtered_cloud(:,ii);
        range = sqrt(sum(cur_point.^2));
        
        % Implementing a circular window 
        window_size = range * res_scale;

        % Extract subset of points within that window
        diff_from_cur_point = filtered_cloud - cur_point;
        dist_from_point = (diff_from_cur_point(1,:).^2 + diff_from_cur_point(2,:).^2).^0.5;
        ind_in_window = find(dist_from_point<window_size);
        heights_in_window = filtered_cloud(3,ind_in_window);

        if cur_point(3) == min(heights_in_window)
            is_peak(ii) = 1;
        elseif cur_point(3) == max(heights_in_window)
            is_peak(ii) = 0;
        end
    end

   peaks_cloud = filtered_cloud(:,find(is_peak));
    
end