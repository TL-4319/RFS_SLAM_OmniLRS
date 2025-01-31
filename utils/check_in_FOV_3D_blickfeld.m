function is_in_FOV_custom = check_in_FOV_3D_blickfeld(rbe, sensor)
    min_range = sensor.min_range;
    max_range = sensor.max_range;
    HFOV = sensor.HFOV; HFOV_bound = 0.5 * (abs(HFOV(1)) + abs(HFOV(end)));
    VFOV = sensor.VFOV;
    
    % Range check
    within_range = [rbe(1,:) > sensor.max_range ; rbe(1,:) < sensor.min_range];
    within_range = ~any(within_range,1); ind_within_range = find(within_range==1);

    rbe_within_range = rbe(:,ind_within_range);
    
    % HFOV check
    within_HFOV = abs(rbe_within_range(2,:)) > HFOV_bound;
    within_HFOV = ~any(within_HFOV,1); ind_within_HFOV = find(within_HFOV==1);
    org_ind_within_HFOV = ind_within_range(ind_within_HFOV);
    
    rbe_within_HFOV = rbe_within_range(:,ind_within_HFOV);

    % VFOV check
    within_VFOV = false(1,size(rbe_within_HFOV,2));
    for ii = 1:size(rbe_within_HFOV,2)
        cur_el_to_test = rbe_within_HFOV(3,ii);
        cur_bearing_to_test = rbe_within_HFOV(2,ii);
        VFOV_bound = interp1(HFOV, abs(VFOV(1,:)), cur_bearing_to_test);
        within_VFOV(ii) = abs(cur_el_to_test) < VFOV_bound;
    end
    ind_within_VFOV = find(within_VFOV==1);
    org_ind_within_VFOV =  org_ind_within_HFOV(ind_within_VFOV);

    is_in_FOV_custom = false(1,size(rbe,2));
    is_in_FOV_custom(org_ind_within_VFOV) = true;

end