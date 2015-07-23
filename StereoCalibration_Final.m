%% Stereo Calibration, rectification, disparity map and 3D point cloud
% Algorithm for perfoming a calibration of a stereo camera system.
% By the use of the estimated camera parameters one can rectify the ...
% stereo images in order to calculate the disparity map of each image ...
% pair.
% Finally a 3D point cloud reconstructs the scene to determine the ...
% distance of object points.

% Author: Maximilian Grams
% Version: 1.07

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
% -> All images have to be seperated into two folders "Left" and ...
% "Right", where the position of the cameras are determined in front ...
% of the system (not the view of the cameras); otherwise you have to ...
% change the disparity range from positive to negative
% -> Every name of an image pair includes the camera direction "R" ...
% (right) or "L" (left) and a number from 1 to numImagePairs, e.g. "R1"
for i = 1:numImagePairs
    imageFileNames1{i} = fullfile(imageDir, 'Right', ...
        sprintf('R%d.jpeg', i));
    imageFileNames2{i} = fullfile(imageDir, 'Left', ...
        sprintf('L%d.jpeg', i));
end

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = ...
    detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Number of image(s) without detected checkerboard points
notUsed = find(ismember(imagesUsed, 0, 'rows'));
% Name all the images without detected checkerboard points
if numImagePairs ~= notUsed
hint = sprintf(['\nWarning: Failed to detect checkerboard points in' ...
    ' stereo pair %d!\n'], notUsed);
disp(hint);
end

% Generate world coordinates of the checkerboard points
squareSize = 25;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = ...
    estimateCameraParameters(imagePoints, worldPoints, ...
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

% Specify 'DisparityRange' of the disparity function by measuring the ...
% horizontal shift
% -> Has to be specified for every distance of the focal plain due to ...
% the dependence of the disparity value from the object distance

% A = stereoAnaglyph(J1{1}, J2{1});
% imtool(A);

% Maximum shift from right to left image: 77 Pixel in image 5
% -> DisparityRange = [0, 80] (must be dividible by 16!)

% Compare undistorted image and disparity map
for i = 1:size(pairsUsed,1)
% Display undistorted image
figureName1 = sprintf(['Image %d: Comparison of rectified image and ' ...
    'disparity map'], i);
figure('Name', figureName1, 'NumberTitle', 'off', 'Units', ...
    'normalized', 'Position', [0, 0, 1, 0.8]);
subplot(1,2,1); imshow(J1{i}); title('Rectified Image', 'FontUnits', ...
    'points', 'FontSize', 15);

% Display disparity map
disparityMap = disparity(rgb2gray(J1{i}), rgb2gray(J2{i}), ...
    'DisparityRange', [0 80], 'Method','SemiGlobal', ...
    'UniquenessThreshold', 5);
subplot(1,2,2); imshow(disparityMap, [0 80], ...
    'InitialMagnification', 50); title('Disparity Map', 'FontUnits', ...
    'points', 'FontSize', 15);
colormap('jet');
colorbar;

% Show the next image by hitting a key or continue with displaying the ...
% 3D reconstruction of the scene by closing a figure
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

% Choose image to be reconstructed
prompt = '\nWhich image would you like to reconstruct? - ';
rightNumber = false;

% Verifying the chosen number
while ~rightNumber
    imageNumber = input(prompt);
    if (imageNumber >= 1) && (imageNumber <= numImagePairs)
        rightNumber = true;
    else
        warning = sprintf(['\nWarning: You can only choose an image' ...
            ' between %d and %d'], 1, numImagePairs);
        disp(warning);
    end
end

% Calculate disparity map of the chosen image
disparityMap = disparity(rgb2gray(J1{imageNumber}), ...
    rgb2gray(J2{imageNumber}), 'DisparityRange', [0 80], 'Method', ...
    'SemiGlobal', 'UniquenessThreshold', 5);

% Reconstruction of disparity map
point3D = reconstructScene(disparityMap, stereoParams);

% Convert from millimeters to meters.
point3D = point3D / 1000;

% Plot points between 1 and 3 meters away from the camera.
z = point3D(:, :, 3);
minZ = 1; % Minimum distance in meters
maxZ = 3; % Maximimum distance in meters
zdisp = z;
% Erase all points outside the distance interval
zdisp(z < minZ | z > maxZ) = NaN; 
point3Ddisp = point3D;
point3Ddisp(:,:,3) = zdisp; % Write back new distance values
figureName2 = sprintf('Image %d: Reconstructed scene from disparity map',...
    imageNumber);
figure('Name', figureName2, 'NumberTitle', 'off', 'Units', 'normalized', ...
    'Position', [0, 0, 1, 0.8]);
showPointCloud(point3Ddisp, J1{imageNumber}, 'VerticalAxis', 'Y',...
    'VerticalAxisDir', 'Down' );
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D point cloud', 'FontUnits', ...
    'points', 'FontSize', 15);