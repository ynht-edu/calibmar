# Changelog

## Unreleased

### New Features

- Kalibr AprilGrid support for marker grid boards: layout preset and `target.yaml` import
- `calibmar_cli` for calibrating from an image directory and a Kalibr AprilGrid `target.yaml` without the GUI
- Import in-air intrinsics from Kalibr `camchain.yaml`/`camchain-imucam.yaml`, ROS `camera_info`, OpenCV and ORB-SLAM3 yaml files in all import dialogs and `calibmar_cli --intrinsics`
- `calibmar_cli --housing` for flat and dome port calibration with fixed in-air intrinsics

### Bugfixes

- Importing a marker grid board calibration now keeps its board origin, ID direction and border bits

## 1.1.0

### New Features

- Add documentation 
- Stereo calibration improvements
  - Support two different camera models
  - Add support for marker grid boards
- Image undistortion dialog
  - Images in a directory can be undistorted given a camera model and parameters
- Interactive housing diagram, allows visualization/exploration of ray geometry for housings in 2D
- Small model explorer improvements

### Bugfixes

- Fix stereo calibrations giving wrong results when cameras had different image sizes
- Skip unreadable files for stereo calibration, which could lead to wrong image pair alignment

## 1.0.0

Initial release