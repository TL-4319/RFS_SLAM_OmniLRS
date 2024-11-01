function [timestamp, se3_transformation] = file2se3(filename)
    data = readmatrix(filename);
    timestamp = data(:,1);

    pos = data(:,2:4);
    quat = quaternion(data(:,5:8));

    se3_transformation = se3(quat, pos);
end