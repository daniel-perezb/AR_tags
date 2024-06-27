clear;
close all;

%% User-defined Variables
filename1 = 'data/Test_10-04-24/LAB1.csv';  % Path to first CSV file
n_markers1 = 6;                            % Number of markers in the first file
filename2 = 'data/Test_10-04-24/LAB1B.csv'; % Path to second CSV file
n_markers2 = 2;                            % Number of markers in the second file

% Output file name
out_file = 'data/Test_10-04-24/transformations.csv'; % Output file

% Master terminal
master = 2;

% Base station number that is in previous test
test_1_station = 2;
test_2_station = 1;

% Invert axis based in TT readings
inv_x = 0;
inv_y = 1;
inv_z = 1;

%% Read and Process Markers from First File
[Points1, Markers1] = readMarkers(filename1, n_markers1);

%% Compute Distances and Transforms for First Set of Markers
[points_2_centre1, points_2_master1] = calculate_transforms(Markers1, master);

%% Read and Process Markers from Second File
[Points2, Markers2] = readMarkers(filename2, n_markers2);

%% Compute Distances and Transforms for Second Set of Markers
[points_2_centre2, points_2_master2] = calculate_transforms(Markers2, test_2_station);

%% Calculate transform between both scans
test_1_points = Points1((test_1_station-1)*4 + 1:test_1_station*4,:);
test_2_points = Points2((test_2_station-1)*4 + 1:test_2_station*4,:);
[R,t] = rigid_transform_3D(test_2_points', test_1_points');
T_matrix = [R,t;0,0,0,1];

% Transform and Merge Marker Sets
Markers1(7:end) = [];

for i = 1:n_markers2
    if i == test_2_station
        continue
    else
        C = [Markers2(i).orientation, Markers2(i).location'; 0, 0, 0, 1];
        Transformed_Points = T_matrix * C;
        Markers1(length(Markers1) + 1).location = Transformed_Points(1:3, 4)';
        Markers1(length(Markers1)).orientation = Transformed_Points(1:3, 1:3);

        if inv_x
            Markers1(length(Markers1)).orientation(:,1) = -Markers1(length(Markers1)).orientation(:,1);
        end
        if inv_y
            Markers1(length(Markers1)).orientation(:,2) = -Markers1(length(Markers1)).orientation(:,2);
        end
        if inv_z
            Markers1(length(Markers1)).orientation(:,3) = -Markers1(length(Markers1)).orientation(:,3);
        end
    
    end
end


%% Plot Merged Markers
plot_markers(Markers1, master);

%% Compute all transforms to master
[points_2_origin, points_2_master] = calculate_transforms(Markers1, master);

for i = 1:size(Markers1,2)
    Markers1(i).distance_to_master = points_2_master(i).distance;
    Markers1(i).transformation_to_master = points_2_master(i).transformation_matrix;
    
    R = points_2_master(i).transformation_matrix(1:3, 1:3);
    % Calculate yaw (ψ) - rotation about the Z-axis
    Markers1(i).yaw = atan2(R(2, 1), R(1, 1));

    % Calculate pitch (θ) - rotation about the Y-axis (after yaw rotation)
    Markers1(i).pitch = asin(-R(3, 1));

    % Calculate roll (φ) - rotation about the X-axis (after yaw and pitch rotations)
    Markers1(i).roll = atan2(R(3, 2), R(3, 3));
end

%% Write output file
% save_markers(Markers1, out_file)

