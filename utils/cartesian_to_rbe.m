function [range, bearing, elevation] = cartesian_to_rbe(cartesian)
    range = vecnorm(cartesian,2,1);

    bearing = atan2(cartesian(2,:), cartesian(1,:));

    r2 = vecnorm(cartesian(1:2,:),2,1);
    elevation = atan2(cartesian(3,:),r2);
end