function [origin_info, point_n_info] = calculate_transforms(Points, n)

% This function calculates the distance and transformation matrix of each
% point to the centre [0,0,0] and to a specific master location

% It expects as input a set of points with specific locaiton and
% orientation
% Points.location = location of point
% Points.orientation = orientation of point
% n = number (int) of master terminal

master_matrix = [horzcat(Points(n).orientation, Points(n).location'); 0, 0, 0, 1];

% Initialize structs for storing information
origin_info = struct('distance', {}, 'transformation_matrix', {});
point_n_info = struct('distance', {}, 'transformation_matrix', {});


% Iterate over all points
for i = 1:length(Points)
    % Distance from origin
    origin_info(i).distance = norm(Points(i).location);

    % Translation matrix from origin to point. THis assumes that the
    % person is at [0,0,0]
    Point_matrix = [horzcat(Points(i).orientation, Points(i).location'); 0, 0, 0, 1]; % 4x4 Matrix
    origin_info(i).transformation_matrix = eye(4) * pinv(Point_matrix);

    % Distance from nth point to current point
    if i ~= n

        point_n_info(i).distance = norm(Points(i).location - Points(n).location);
        % Translation matrix from nth point to current point
        point_n_info(i).transformation_matrix = master_matrix * pinv(Point_matrix);

    else
        point_n_info(i).distance = 0; % Distance from itself
        point_n_info(i).transformation_matrix = eye(4);
    end
end
