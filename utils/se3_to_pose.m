function [pos, quat] = se3_to_pose(se3_mat)
    
    pos = squeeze(se3_mat(1:3,4,:));

    rotm = se3_mat(1:3,1:3,:);
    quat = quaternion(rotm,'rotmat','point');
end