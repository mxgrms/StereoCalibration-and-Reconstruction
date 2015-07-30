# StereoCalibration
MATLAB algorithm for calibrating a stereo camera system, displaying a disparity map, and doing a 3D reconstruction of the captured scene.

## Functions
- [x] Reading the input image pairs by adding them to a defined folder structure in a specfied working directory
- [x] Calibrating the stereo camera system by estimating the intrinsic and extrinsic parameters, and the coefficients of the radial distortion
- [x] Displaying the reprojection error of each view and the resulting averge error in order to verify the quality of the estimated camera model
- [x] Calculating the disparity map for each stereo image pair
- [x] Reconstructing the captured scene as a point cloud

## TODO
- [ ] Separate the calibration algorithm from the actual reconstruction part in order to allow the reconstruction of a every captured image pair based on the calibration you executed once (at the moment you reconstruct the images you use for the calibration)
- [ ] Creating a GUI that allows a user to easily set up a stereo camera system and use the software to verify the achievements
