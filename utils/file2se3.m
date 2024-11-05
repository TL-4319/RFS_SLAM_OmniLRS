function [timestamp, se3_out] = file2se3(filename)
    data = readmatrix(filename);
    timestamp = data(:,1);

    %pos = data(:,2:4);
    %quat = quaternion(data(:,5:8));

    %se3_transformation = se3(quat, pos);
    
    se3_out = zeros(4,4,size(timestamp,1));
    for ii = 1:size(timestamp,1)
        cur_quat = data(ii,5:8);
        cur_pos = transpose(data(ii,2:4));
        
        cur_se3 = eye(4);
        cur_se3(1:3,4) = cur_pos;
        cur_se3(1:3,1:3) = quat2rotm(cur_quat);
        se3_out(:,:,ii) = cur_se3;
    end
end