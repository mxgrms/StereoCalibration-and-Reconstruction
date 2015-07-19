%% Stereo Calibration, rectification, disparity map and 3D point cloud
% Algorithm for perfoming a calibration of a stereo camera system.
% By the use of the estimated camera parameters one can rectify the stereo
% images in order to calculate the disparity map of each image pair.
% Finally a 3D point cloud reconstructs the scene to determine the distance
% of object points.

%% Stereo Calibration

% Number of input images
numImagePairs = 10;
% Directory of MATLAB workspace
workspaceDir = '/Users/maximiliangrams/Documents/MATLAB';
imageFileNames1 = cell(numImagePairs, 1);
imageFileNames2 = cell(numImagePairs, 1);
% Directory of stereo images
imageDir = fullfile(workspaceDir, 'StereoCalibration', 'Grossbild');

% Filenames
for i = 1:numImagePairs
    imageFileNames1{i} = fullfile(imageDir, 'Left',  sprintf('L%d.jpeg', i));
    imageFileNames2{i} = fullfile(imageDir, 'Right', sprintf('R%d.jpeg', i));
end

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Number of picture(s) without detected checkerboard points
notUsed = find(ismember(imagesUsed, 0, 'rows'));
if numImagePairs ~= notUsed
hint = sprintf('\nWarning: Failed to detect checkerboard points in stereo pair %d!\n', notUsed);
disp(hint);
end

% Generate world coordinates of the checkerboard keypoints
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm');

% View reprojection errors
h1=figure; showReprojectionErrors(stereoParams, 'BarGraph');

% Visualize pattern locations
h2=figure; showExtrinsics(stereoParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, stereoParams);


%% Rectification and disparity map

% Initialize image cells
I1 = cell(1, numImagePairs);
I2 = cell(1, numImagePairs);
J1 = cell(1, numImagePairs);
J2 = cell(1, numImagePairs);

% Correct distortion and rectify images
for i = 1:numImagePairs
I1{i} = imread(imageFileNames1{i});
I2{i} = imread(imageFileNames2{i});
[J1{i}, J2{i}] = rectifyStereoImages(I1{i}, I2{i}, stereoParams);
end

for i = 1:size(pairsUsed,1)
%Display undistorted image
figure('Name', 'Comparison of rectified image and disparity map', ...
    'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0, 0, 1, 0.8]);
subplot(1,2,1); imshow(J1{i}); title('Rectified Image', 'FontUnits', ...
    'points', 'FontSize', 15);

% Display disparity map
disparityMap = disparity(rgb2gray(J1{i}), rgb2gray(J2{i}), ...
    'DisparityRange', [-96 0], 'Method','SemiGlobal', ...
    'UniquenessThreshold', 5, 'DistanceThreshold', 40);
subplot(1,2,2); imshow(disparityMap, [-96 0], ...
    'InitialMagnification', 50); title('Disparity Map', 'FontUnits', ...
    'points', 'FontSize', 15);
colormap('jet');
colorbar;

try
if waitforbuttonpress
    close all
elseif not(waitforbuttonpress)
    pause % Enable processing the figure
end
catch
    break % Stop plotting results if one figure was closed
end

close all % Close figure if key was not hit

end

%% 3D point cloud

% Choose picture to be reconstructed
prompt = '\nWhich picture would you like to reconstruct? - ';
rightNumber = false;

% Verifying the chosen number
while ~rightNumber
    imageNumber = input(prompt);
    if (imageNumber >= 1) && (imageNumber <= numImagePairs)
        rightNumber = true;
    else
        warning = sprintf('\nWarning: You can only choose a picture between %d and %d\n',...
        1, numImagePairs);
        disp(warning);
    end
end

disparityMap = disparity(rgb2gray(J1{imageNumber}), rgb2gray(J2{imageNumber}), ...
    'DisparityRange', [-96 0], 'Method','SemiGlobal', ...
    'UniquenessThreshold', 5, 'DistanceThreshold', 40);

% Reconstruction of disparity map
point3D = reconstructScene(disparityMap, stereoParams);

% Convert from millimeters to meters.
point3D = point3D / 1000;

% Plot points between 0 and 2 meters away from the camera.
z = point3D(:, :, 3);
minZ = 0; % Minimum distance in meters
maxZ = 2; % Maximimum distance in meters
zdisp = z;
zdisp(z < minZ | z > maxZ) = NaN; % Erase all points outside the distance interval
point3Ddisp = point3D;
point3Ddisp(:,:,3) = zdisp; % Write back new distance values
figure('Name', 'Reconstructed scene from disparity map', ...
    'NumberTitle', 'off', 'Units', 'normalized', 'Position', [0, 0, 1, 0.8]); ...
showPointCloud(point3Ddisp, J1{imageNumber}, 'VerticalAxis', 'Y',...
    'VerticalAxisDir', 'Down' );
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D point cloud', 'FontUnits', ...
    'points', 'FontSize', 15);