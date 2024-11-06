function peaks_cloud = detect_peak(input_cloud, res_scale)
    is_peak = zeros(1,size(input_cloud,2));

    for ii = 1:size(input_cloud,2)
        cur_point = input_cloud(:,ii);
        range = sqrt(sum(cur_point.^2));
        
        % Implementing a circular window 
        window_size = range * res_scale;

        % Extract subset of points within that window
        diff_from_cur_point = input_cloud - cur_point;
        dist_from_point = (diff_from_cur_point(1,:).^2 + diff_from_cur_point(2,:).^2).^0.5;
        ind_in_window = find(dist_from_point<window_size);
        heights_in_window = input_cloud(3,ind_in_window);

        if cur_point(3) == max(heights_in_window)
            is_peak(ii) = 1;
        elseif cur_point(3) == min(heights_in_window)
            is_peak(ii) = 1;
        end
    end

   peaks_cloud = input_cloud(:,find(is_peak));
    
end