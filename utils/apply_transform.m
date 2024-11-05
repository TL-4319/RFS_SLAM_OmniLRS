function rotated_points = apply_transform(se3, points)
    % se3 are typical se3 transformation matrix
    % points are 3xN
    homo_points = vertcat(points,ones(1,size(points,2)));
    
    % Pre allocate array
    rotated_points = homo_points;

    for ii = 1:size(homo_points,2)
        rotated_points(:,ii) = se3 * homo_points(:,ii);
    end
    
    % Convert from homogeneous rep to cartesian rep
    rotated_points(4,:) = [];
end