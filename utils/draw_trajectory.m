function draw_trajectory (se3, se3_hist, scale, linewidth,traj_color,bool_hold)
    
    rotm = se3(1:3,1:3);

    pos = se3(1:3,4);

    traj_hist = squeeze(se3_hist(1:3,4,:));
    
    % Rotate of principle axis
    rot_x = rotm * [1;0;0] * scale;
    rot_y = rotm * [0;1;0] * scale;
    rot_z = rotm * [0;0;1] * scale;

    unit_x = horzcat (pos, pos + rot_x);
    unit_y = horzcat (pos, pos + rot_y);
    unit_z = horzcat (pos, pos + rot_z);

    %figure (fig_num)
    if bool_hold
        hold on
    else
        hold off
    end
    plot3 (unit_x(1,:), unit_x(2,:),unit_x(3,:), 'r','LineWidth',linewidth, 'HandleVisibility','off')
    hold on
    plot3 (unit_y(1,:), unit_y(2,:),unit_y(3,:), 'g','LineWidth',linewidth,'HandleVisibility','off')
    plot3 (unit_z(1,:), unit_z(2,:),unit_z(3,:), 'b', 'LineWidth',linewidth,'HandleVisibility','off')
    plot3 (traj_hist(1,:), traj_hist (2,:), traj_hist(3,:), traj_color,'HandleVisibility','off')
    hold off
end