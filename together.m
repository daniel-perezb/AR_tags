clear;
close all;

% User-defined Variables
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

% Read and Process Markers from First File
[Points1, Markers1] = readMarkers(filename1, n_markers1);

% Compute Distances and Transforms for First Set of Markers
[points_2_centre1, points_2_master1] = calculate_transforms(Markers1, master);

% Read and Process Markers from Second File
[Points2, Markers2] = readMarkers(filename2, n_markers2);

% Compute Distances and Transforms for Second Set of Markers
[points_2_centre2, points_2_master2] = calculate_transforms(Markers2, test_2_station);

% Calculate transform between both scans
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

plot_markers(Markers1, master);
%% Main file

% Variables
folder_name = 'data/Extracted_Images_2';
files = dir(fullfile(folder_name, '*.jpg'));
files = files(~[files.isdir]);

camParamMatFilePath = "./calibration/cameraParamsIp14Pro.mat";
markerSizeMM = 180; % Ar tag marker size in mm

% Marker index based on images taken
% Marker_Image = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2];
Marker_Image = [2, 4, 2, 4, 2, 4];

% Initialise variables
cameraPositions = zeros(size(files, 1), 3);
cameraOrientations = zeros(3, 3, size(files, 1));
angles = zeros(size(files, 1),3);
final_angles = zeros(size(files, 1),1);
Initial = eye(3);
change_axes = 1;

% Scale factor for visualizing the axes
axisLength = 0.2;

people = struct('location', {}, 'orientation', {});

for i = 1:size(files,1)

    % Get pose of the marker in image
    arTagImagePath = fullfile(folder_name, files(i).name);
    [~, pose, ~, ~] = getPose(arTagImagePath, camParamMatFilePath, markerSizeMM);
    
    % If Image cannot detect any marker
    if size(pose,2) == 0
        continue
    end
    % Extract the translation vector and rotation matrix
    translationVector = pose.A(1:3, 4) * 0.001; % Convert units from mm to meters
    rotationMatrix = pose.A(1:3, 1:3);

    if (change_axes)
           tmp = translationVector(1);
    translationVector(1) = translationVector(3);
    translationVector(3) = tmp;
    end

    % Find rotationj on x axis
    R_AB = rotationMatrix * eye(3);
    eulerAngle = rotm2eul(R_AB, 'XYZ');
    angles(i,:) = rad2deg(eulerAngle);

    % Angle difference in degrees
    theta_rad = deg2rad(180 - abs(angles(i,1)));
    final_angles(i) = rad2deg(theta_rad);

    % Rotation matrix
    Rx = [1, 0, 0;
        0, cos(theta_rad), -sin(theta_rad);
        0, sin(theta_rad), cos(theta_rad)];

    % Update location of camera position
    % Below changes the location of origin, this needs to be checked for
    % orientation 
        cameraPositions(i,:) = translationVector ;
    cameraOrientations(:, :, i) =  rotationMatrix;
        people(end+1).location = cameraPositions(i,:);
    people(end).orientation = cameraOrientations(:, :, i);
    cameraPositions(i,:) = (translationVector + Rx * (-translationVector))';
    cameraOrientations(:, :, i) = Rx * rotationMatrix;

    people(end+1).location = cameraPositions(i,:);
    people(end).orientation = cameraOrientations(:, :, i);




    
end

% Plot location of people and markers
plot_markers_people(Markers1, master,people);
