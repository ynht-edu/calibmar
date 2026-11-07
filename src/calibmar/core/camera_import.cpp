#include "calibmar/core/camera_import.h"
#include "calibmar/core/simple_yaml.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace {
  using namespace calibmar;
  using camera_import::ImportedCamera;
  using namespace simple_yaml;

  std::pair<int, int> Resolution(const Values& values, const std::string& width_key, const std::string& height_key) {
    std::optional<double> width = OptionalNumber(values, width_key);
    std::optional<double> height = OptionalNumber(values, height_key);
    if (width && height) {
      return {static_cast<int>(*width), static_cast<int>(*height)};
    }
    return {0, 0};
  }

  bool NearZero(double v) {
    return std::abs(v) < 1e-12;
  }

  // Maps OpenCV style distortion coefficients (k1, k2, p1, p2[, k3[, k4, k5, k6[, ...]]]) to a calibmar model.
  void SetPinholeDistortion(ImportedCamera& camera, std::vector<double> coeffs) {
    while (coeffs.size() > 8 && NearZero(coeffs.back())) {
      coeffs.pop_back();
    }
    if (coeffs.empty() || std::all_of(coeffs.begin(), coeffs.end(), NearZero)) {
      camera.camera_model = CameraModelType::PinholeCameraModel;
      return;
    }
    if (coeffs.size() < 4) {
      coeffs.resize(4, 0.0);
    }
    if (coeffs.size() == 4 || (coeffs.size() == 5 && NearZero(coeffs[4]))) {
      camera.camera_model = CameraModelType::OpenCVCameraModel;
      camera.params.insert(camera.params.end(), coeffs.begin(), coeffs.begin() + 4);
      return;
    }
    if (coeffs.size() > 8) {
      throw std::runtime_error(camera.name + ": thin prism / tilted distortion (" + std::to_string(coeffs.size()) +
                               " coefficients) is not supported");
    }
    coeffs.resize(8, 0.0);
    camera.camera_model = CameraModelType::FullOpenCVCameraModel;
    camera.params.insert(camera.params.end(), coeffs.begin(), coeffs.end());
  }

  void SetFisheyeDistortion(ImportedCamera& camera, const std::vector<double>& coeffs) {
    if (coeffs.size() != 4) {
      throw std::runtime_error(camera.name + ": fisheye distortion needs 4 coefficients, got " +
                               std::to_string(coeffs.size()));
    }
    camera.camera_model = CameraModelType::OpenCVFisheyeCameraModel;
    camera.params.insert(camera.params.end(), coeffs.begin(), coeffs.end());
  }

  ImportedCamera MakeCamera(const std::string& name, const std::string& format, double fx, double fy, double cx, double cy) {
    ImportedCamera camera;
    camera.name = name;
    camera.format = format;
    camera.params = {fx, fy, cx + 0.5, cy + 0.5};
    return camera;
  }

  std::vector<ImportedCamera> ParseKalibr(const Values& values) {
    std::vector<ImportedCamera> cameras;
    for (const auto& [key, value] : values) {
      const std::string suffix = ".camera_model";
      if (key.size() <= suffix.size() || key.compare(key.size() - suffix.size(), suffix.size(), suffix) != 0) {
        continue;
      }
      std::string name = key.substr(0, key.size() - suffix.size());
      if (name.find('.') != std::string::npos || !Has(values, name + ".intrinsics")) {
        continue;
      }

      std::string projection = Text(values, name + ".camera_model");
      std::string distortion = Text(values, name + ".distortion_model", "none");
      if (projection != "pinhole") {
        throw std::runtime_error("Kalibr " + name + ": camera_model '" + projection + "' is not supported, only pinhole");
      }
      std::vector<double> k = List(values, name + ".intrinsics");
      if (k.size() != 4) {
        throw std::runtime_error("Kalibr " + name + ": intrinsics must have 4 values");
      }

      ImportedCamera camera = MakeCamera(name, "Kalibr camchain", k[0], k[1], k[2], k[3]);
      std::vector<double> coeffs =
          distortion == "none" ? std::vector<double>{} : List(values, name + ".distortion_coeffs");
      if (distortion == "radtan") {
        if (coeffs.size() != 4) {
          throw std::runtime_error("Kalibr " + name + ": radtan needs 4 distortion_coeffs");
        }
        camera.camera_model = CameraModelType::OpenCVCameraModel;
        camera.params.insert(camera.params.end(), coeffs.begin(), coeffs.end());
      }
      else if (distortion == "equidistant") {
        SetFisheyeDistortion(camera, coeffs);
      }
      else if (distortion == "none") {
        camera.camera_model = CameraModelType::PinholeCameraModel;
      }
      else {
        throw std::runtime_error("Kalibr " + name + ": distortion_model '" + distortion +
                                 "' is not supported, only radtan, equidistant or none");
      }

      if (Has(values, name + ".resolution")) {
        std::vector<double> resolution = List(values, name + ".resolution");
        if (resolution.size() == 2) {
          camera.resolution = {static_cast<int>(resolution[0]), static_cast<int>(resolution[1])};
        }
      }
      cameras.push_back(camera);
    }
    return cameras;
  }

  std::vector<ImportedCamera> ParseOrbSlam3(const Values& values) {
    std::vector<std::string> prefixes;
    for (const std::string& prefix : {"Camera1", "Camera2"}) {
      if (Has(values, prefix + ".fx")) {
        prefixes.push_back(prefix);
      }
    }
    if (prefixes.empty() && Has(values, "Camera.fx")) {
      prefixes.push_back("Camera");
    }

    std::string type = Text(values, "Camera.type", "PinHole");
    std::vector<ImportedCamera> cameras;
    for (const auto& prefix : prefixes) {
      ImportedCamera camera = MakeCamera(prefix, "ORB-SLAM3", Number(values, prefix + ".fx"), Number(values, prefix + ".fy"),
                                         Number(values, prefix + ".cx"), Number(values, prefix + ".cy"));
      if (type == "KannalaBrandt8") {
        SetFisheyeDistortion(camera, {Number(values, prefix + ".k1"), Number(values, prefix + ".k2"),
                                      Number(values, prefix + ".k3"), Number(values, prefix + ".k4")});
      }
      else if (type == "PinHole" || type == "Rectified") {
        std::vector<double> coeffs;
        for (const std::string& k : {"k1", "k2", "p1", "p2", "k3"}) {
          coeffs.push_back(OptionalNumber(values, prefix + "." + k).value_or(0.0));
        }
        SetPinholeDistortion(camera, coeffs);
      }
      else {
        throw std::runtime_error("ORB-SLAM3 Camera.type '" + type + "' is not supported");
      }
      camera.resolution = Resolution(values, "Camera.width", "Camera.height");
      cameras.push_back(camera);
    }
    return cameras;
  }

  // ROS camera_info (camera_calibration_parsers) and OpenCV FileStorage calibration files
  std::vector<ImportedCamera> ParseMatrixFormat(const Values& values) {
    std::optional<std::string> k_key =
        FirstKey(values, {"camera_matrix.data", "cameraMatrix.data", "K.data", "intrinsic_matrix.data"});
    if (!k_key) {
      return {};
    }
    std::vector<double> k = List(values, *k_key);
    if (k.size() != 9) {
      throw std::runtime_error("Camera matrix must have 9 values");
    }

    bool is_ros = Has(values, "distortion_model") || Has(values, "camera_name");
    std::string name = Text(values, "camera_name", "camera");
    ImportedCamera camera = MakeCamera(name, is_ros ? "ROS camera_info" : "OpenCV", k[0], k[4], k[2], k[5]);

    std::optional<std::string> d_key = FirstKey(values, {"distortion_coefficients.data", "dist_coeffs.data",
                                                         "distCoeffs.data", "D.data", "distortion_coeffs.data"});
    std::vector<double> coeffs = d_key ? List(values, *d_key) : std::vector<double>{};

    std::string model = Text(values, "distortion_model");
    bool fisheye = model == "equidistant" || model == "fisheye";
    for (const std::string& flag : {"fisheye_model", "Calibrate_UseFisheyeModel", "use_fisheye"}) {
      if (Has(values, flag) && Text(values, flag) != "0" && Text(values, flag) != "false") {
        fisheye = true;
      }
    }

    if (fisheye) {
      SetFisheyeDistortion(camera, coeffs);
    }
    else if (model.empty() || model == "plumb_bob" || model == "rational_polynomial") {
      SetPinholeDistortion(camera, coeffs);
    }
    else {
      throw std::runtime_error("distortion_model '" + model + "' is not supported");
    }

    camera.resolution = Resolution(values, "image_width", "image_height");
    if (camera.resolution.first == 0) {
      camera.resolution = Resolution(values, "width", "height");
    }
    return {camera};
  }
}

namespace calibmar {
  namespace camera_import {
    std::vector<ImportedCamera> ParseCameras(const std::string& content) {
      Values values = simple_yaml::Parse(content);
      for (auto parser : {ParseKalibr, ParseOrbSlam3, ParseMatrixFormat}) {
        std::vector<ImportedCamera> cameras = parser(values);
        if (!cameras.empty()) {
          return cameras;
        }
      }
      throw std::runtime_error(
          "No camera found. Supported are Kalibr camchain, ROS camera_info, OpenCV and ORB-SLAM3 yaml files.");
    }

    bool IsSupported(const std::string& content) {
      Values values = simple_yaml::Parse(content);
      if (FirstKey(values, {"camera_matrix.data", "cameraMatrix.data", "K.data", "intrinsic_matrix.data", "Camera.fx",
                            "Camera1.fx"})) {
        return true;
      }
      for (const auto& [key, value] : values) {
        size_t dot = key.find('.');
        if (dot != std::string::npos && key.substr(dot) == ".camera_model" &&
            Has(values, key.substr(0, dot) + ".intrinsics")) {
          return true;
        }
      }
      return false;
    }

    bool IsSupportedFile(const std::string& path) {
      try {
        return IsSupported(simple_yaml::ReadFile(path));
      }
      catch (const std::exception&) {
        return false;
      }
    }

    std::vector<ImportedCamera> ReadCameras(const std::string& path) {
      return ParseCameras(simple_yaml::ReadFile(path));
    }
  }
}
