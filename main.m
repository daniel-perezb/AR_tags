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

%% Read and Process Markers from First File
[Points1, Markers1] = readMarkers(filename1, n_markers1);

%% Compute Distances and Transforms for First Set of Markers
[points_2_centre1, points_2_master1] = calculate_transforms(Markers1, master);

%% Read and Process Markers from Second File
[Points2, Markers2] = readMarkers(filename2, n_markers2);

%% Compute Distances and Transforms for Second Set of Markers
[points_2_centre2, points_2_master2] = calculate_transforms(Markers2, test_2_station);

%% Extract Ground Truth Stations for Both Scans
A = [horzcat(Markers1(test_1_station).orientation, Markers1(test_1_station).location'); 0, 0, 0, 1];
B = [horzcat(Markers2(test_2_station).orientation, Markers2(test_2_station).location'); 0, 0, 0, 1];

Trans = A * pinv(B);

%% Transform and Merge Marker Sets
xc = Trans * B;
Markers1(7:end) = [];

for i = 1:n_markers2
    if i == test_2_station
        continue
    else
        j = size(Markers1, 2);
        C = [horzcat(Markers2(i).orientation, Markers2(i).location'); 0, 0, 0, 1];
        Transformed_Points = Trans * C;
        Markers1(j + 1).location = Transformed_Points(1:3, 4)';
        Markers1(j + 1).orientation = Transformed_Points(1:3, 1:3);
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
save_markers(Markers1, out_file)

%% Optional Testing Functions (commented out)
% % Function for loading data and testing
% load('data/Test_10-04-24/IMG_4692.mat')
% poses.Translation = poses.Translation * .001; % Transform to meters
% human_location = norm([0, 0, 0] - poses.Translation); % Check distance from origin (phone)
% difference_human_camera = norm(human_location - points_2_master1(m).distance); % Error between theodolite and camera

% % Function to read extra points (commented out)
% data1 = readtable(filename1, 'FileType', 'text', 'VariableNamingRule', 'preserve', 'Delimiter', ',');
% data2 = readtable(filename2, 'FileType', 'text', 'VariableNamingRule', 'preserve', 'Delimiter', ',');
% for i = 1:2
%     A(:, end + 1) = [data1.Var2(i + 24), data1.Var3(i + 24), data1.Var4(i + 24)]' - [data1.Var2(end), data1.Var3(end), data1.Var4(end)]';
%     B(:, end + 1) = [data2.Var2(i + 8), data2.Var3(i + 8), data2.Var4(i + 8)]' - [data2.Var2(end), data2.Var3(end), data2.Var4(end)]';
% end

% % Optimal rotation and translation between both sets of points (commented out)
% B = Points1(5:8, :)';
% A = Points2(1:4, :)';
% [R, t] = rigid_transform_3D(A, B);
% transformation_matrix = [R, t; 0, 0, 0, 1];
% Markers1(7:end) = [];
% for i = 1:n_markers2
%     j = size(Markers1, 2);
%     C = [horzcat(Markers2(i).orientation, Markers2(i).location'); 0, 0, 0, 1];
%     Transformed_Points = transformation_matrix * C;
%     Markers1(j + 1).location = Transformed_Points(1:3, 4)';
%     Markers1(j + 1).orientation = Transformed_Points(1:3, 1:3);
% end
% plot_markers(Markers1, master1);
