function noisy_cloud = add_pc_noise(cloud, lidar_noise)
    rbe = cart2rbe(cloud);

    rbe(:,1) = rbe(:,1) + normrnd(0, lidar_noise.range_m, size(rbe(:,1)));
    rbe(:,2:3) = rbe(:,2:3) + normrnd(0, deg2rad(lidar_noise.angular_deg), size(rbe(:,2:3)));

    noisy_cloud = rbe2cart(rbe);
    
end

function rbe = cart2rbe(cart)
    [az, el, r] = cart2sph(cart(:,1), cart(:,2),cart(:,3));
    rbe = [r, az, el];
end

function cart = rbe2cart(rbe)
    [x, y, z] = sph2cart(rbe(:,2), rbe(:,3), rbe(:,1));
    cart = [x,y,z];
end