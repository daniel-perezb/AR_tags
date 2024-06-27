% clear
function [I, poses, unixEpoch, timeString] = getPose(arTagImagePath, camParamMatFilePath, markerSizeMM)
% arTagImagePath = "./data/Test_16-04-24/IMG_4704.jpg"; % Multiple tags
%arTagImagePath = "./images/ar/785mm_v2.jpg"; % Single ar tag at 785mm distance (top down)
% camParamMatFilePath = "./calibration/cameraParamsIp14Pro.mat";
% markerSizeMM = 180; % Ar tag marker size in mm

%% Locading cam intrinsics
% Create this mat file using cameraCalibrator App and save it. Info: https://au.mathworks.com/help/vision/ref/cameracalibrator-app.html
data = load(camParamMatFilePath);
% This data.cameraParams depends on how you saved the mat file and change
% as necessary.
intrinsics = data.cameraParams.Intrinsics;

%% Check tag family can be detected for multiple markers and overlay detected markers
% This will print the ar tag ID and family
% I = checkArTagInfo(ariTagImagePath); % Replace this with a image path
% imshow(I)

% Get the image (as object), pose as 4 x 4 matrix, time as epoch (seconds)
% and time as string (yyyy:MM:dd HH:mm:ss)
[I, poses, unixEpoch, timeString] = getArTagPoseAndTime(arTagImagePath, intrinsics, markerSizeMM);

% plotArTagPoseOnImg(markerSizeMM, poses, intrinsics, I);

end

function I = checkArTagInfo(imgPath)
    %  CHECKARTAGINFO Take an image of AR tag as an argument and print ID and
    %  family (e.g. DICT_4x4_1000) of it. Image can have multiple markers.

    I = imread(imgPath);
    [ids, locs, detectedFamily] = readArucoMarker(I);

    numMarkers = length(ids);

    for i = 1:numMarkers
        loc = locs(:, :, i);

        % Display the marker ID and family
        disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))

        % Insert marker edges
        I = insertShape(I, "polygon", {loc}, Opacity = 1, ShapeColor = "green", LineWidth = 4);

        % Insert marker corners
        markerRadius = 6;
        numCorners = size(loc, 1);
        markerPosition = [loc, repmat(markerRadius, numCorners, 1)];
        I = insertShape(I, "FilledCircle", markerPosition, ShapeColor = "red", Opacity = 1);

        % Insert marker IDs
        center = mean(loc);
        I = insertText(I, center, ids(i), FontSize = 30, BoxOpacity = 1);
    end

end

function plotArTagPoseOnImg(markerSizeMM, poses, camIntrinsics, I)
    % Origin and axes vectors for the object coordinate system
    worldPoints = [0 0 0; markerSizeMM / 2 0 0; 0 markerSizeMM / 2 0; 0 0 markerSizeMM / 2];

    % Plotting origin for visualisation
    for i = 1:length(poses)
        % Get image coordinates for axes
        imagePoints = world2img(worldPoints, poses(i), camIntrinsics);

        axesPoints = [imagePoints(1, :) imagePoints(2, :);
                      imagePoints(1, :) imagePoints(3, :);
                      imagePoints(1, :) imagePoints(4, :)];
        % Draw colored axes
        I = insertShape(I, "Line", axesPoints, ...
            Color = ["red", "green", "blue"], LineWidth = 10);
    end
    figure
    imshow(I)
end

function [I, poses, unixEpoch, timeString] = getArTagPoseAndTime(arTagImagePath, intrinsics, markerSizeMM)
    I = imread(arTagImagePath);
    % [~, ~, detectedFamily] = readArucoMarker(I);
    detectedFamily = "DICT_4X4_1000";
    markerFamily = detectedFamily(1); % Assuming all markers are same type. Or you can hard code it for faster results.

    % [I, ~] = undistortImage(I, intrinsics);
    [~, ~, poses] = readArucoMarker(I, markerFamily, intrinsics, markerSizeMM);

    % date time as a string (yyyy:MM:dd HH:mm:ss). Note
    %  that this only has been tested with iPhone images. Not sure if other
    %  phones has different time format or parameters. Time zone is set to the
    %  local time zone of the machine that run this script. If there are differences between
    %  image taken and current timezone of the script run, pleace change the
    %  TimeZone parameter.
    info = imfinfo(arTagImagePath);
    timeString = info.DateTime; % String in the format of 'yyyy:MM:dd HH:mm:ss'
    dt = datetime(timeString, 'InputFormat', 'yyyy:MM:dd HH:mm:ss', 'TimeZone', 'local'); % Converting to an object with local timezone.
    unixTime = posixtime(dt); % In seconds
    unixEpoch = unixTime;
end
