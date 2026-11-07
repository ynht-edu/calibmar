#pragma once

#include "calibmar/core/camera_models.h"

#include <string>
#include <utility>
#include <vector>

namespace calibmar {
  // Import of camera intrinsics calibrated by other tools:
  //  - Kalibr camchain.yaml / camchain-imucam.yaml (pinhole with radtan, equidistant or no distortion)
  //  - ROS camera_info yaml (plumb_bob, rational_polynomial, equidistant)
  //  - OpenCV FileStorage yaml (camera_matrix + distortion_coefficients)
  //  - ORB-SLAM3 settings yaml (PinHole, KannalaBrandt8, Rectified)
  // All of these use pixel centers at integer coordinates, so the principal point is shifted by +0.5 px
  // to the calibmar/COLMAP convention.
  namespace camera_import {
    struct ImportedCamera {
      std::string name;
      std::string format;
      CameraModelType camera_model;
      std::vector<double> params;
      std::pair<int, int> resolution = {0, 0};
    };

    bool IsSupported(const std::string& content);
    bool IsSupportedFile(const std::string& path);

    std::vector<ImportedCamera> ParseCameras(const std::string& content);
    std::vector<ImportedCamera> ReadCameras(const std::string& path);
  }
}
