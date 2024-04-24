clear;
close all
%% Main file

% Path to CSV file
filename = 'data/Test_10-04-24/LAB1.csv'; 
n_markers = 6;
[Points, Markers] = readMarkers(filename, n_markers);

% Function below to change heigth of a specific marker
m = 3;
Markers(m).location = Markers(m).location + [0,0,1.3];



%% Compute distances between points

% Number of master terminal
master = 2;

% Function to compute transformation matrices between the centre and the
% other points and master and the other points
[points_2_centre, points_2_master] = calculate_transforms(Markers,master);


%% Plot markers
plot_markers(Markers, master);

%% Second set of markers

% Path to CSV file
filename2 = 'data/Test_10-04-24/LAB1B.csv'; 
n_markers2 = 2;
[Points2, Markers2] = readMarkers(filename2, n_markers2);

% Number of master terminal
master2 = 1;

% Calculate transforms
[points_2_centre2, points_2_master2] = calculate_transforms(Markers2,master2);

plot_markers(Markers2, master2);


%% Extract the ground trith station for both scans
A = [horzcat(Markers(master).orientation, Markers(master).location'); 0, 0, 0, 1]; 
B = [horzcat(Markers2(master2).orientation, Markers2(master2).location'); 0, 0, 0, 1]; 

Trans = A * pinv(B);
xc = Trans * B; 


Markers(7:end) = [];

for i = 1:n_markers2
j = size(Markers,2);
C = [horzcat(Markers2(i).orientation, Markers2(i).location'); 0,0,0,1];
Transformed_Points =  Trans * C;
Markers(j+1).location = Transformed_Points(1:3,4)';
Markers(j+1).orientation = Transformed_Points(1:3,1:3);
end
















%% Functions below are for testing
% load('data/Test_10-04-24/IMG_4692.mat')
% poses.Translation = poses.Translation * .001; % Transform to m
% human_location =  norm([0,0,0] - poses.Translation); % Check how far away it is from origin (phone)
% difference_human_camera = norm(human_location - points_2_master(m).distance) % Erro between theodolite and camera


% % Used to read the extra points
% % Problem with one value being incorrect symbol
% data1 = readtable(filename, 'FileType', 'text', 'VariableNamingRule', 'preserve', 'Delimiter', ',');
% data2 = readtable(filename2, 'FileType', 'text', 'VariableNamingRule', 'preserve', 'Delimiter', ',');
% 
% for i=1:2
%     A(:,end+1) = [data1.Var2(i + 24), data1.Var3(i + 24), data1.Var4(i + 24)]' - [data1.Var2(end), data1.Var3(end), data1.Var4(end)]';
%     B(:,end+1) = [data2.Var2(i + 8), data2.Var3(i + 8), data2.Var4(i + 8)]' - [data2.Var2(end), data2.Var3(end), data2.Var4(end)]' ;
% 
% end






%% Optimal rotation and translation between both set of points
% % Extract the points of the master in both scans
% B = Points(5:8,:)';
% A = Points2(1:4,:)';
% 
% % Compute the transformation matrix
% [R,t] = rigid_transform_3D(A, B);
% transformation_matrix = [R, t; 0, 0, 0, 1];
% 
% 
% Markers(7:end) = [];
% 
% % Multiply points by translation and rotation matrix
% for i = 1:n_markers2
% j = size(Markers,2);
% C = [horzcat(Markers2(i).orientation, Markers2(i).location'); 0,0,0,1];
% Transformed_Points =  transformation_matrix * C;
% Markers(j+1).location = Transformed_Points(1:3,4)';
% Markers(j+1).orientation = Transformed_Points(1:3,1:3);
% end
% 
% plot_markers(Markers, master);
% 
% 
% 
