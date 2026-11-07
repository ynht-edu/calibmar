#include "calibmar/core/camera_import.h"

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
  using Values = std::map<std::string, std::string>;

  std::string Trim(const std::string& s) {
    auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
    auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return begin < end ? std::string(begin, end) : std::string();
  }

  std::string StripComment(const std::string& line) {
    char quote = 0;
    for (size_t i = 0; i < line.size(); i++) {
      char c = line[i];
      if (quote) {
        if (c == quote) quote = 0;
      }
      else if (c == '\'' || c == '"') {
        quote = c;
      }
      else if (c == '#' && (i == 0 || std::isspace(static_cast<unsigned char>(line[i - 1])))) {
        return line.substr(0, i);
      }
    }
    return line;
  }

  std::string Unquote(const std::string& s) {
    if (s.size() >= 2 && (s.front() == '\'' || s.front() == '"') && s.back() == s.front()) {
      return s.substr(1, s.size() - 2);
    }
    return s;
  }

  // Flattens nested yaml mappings to dotted keys. Block sequences are skipped, flow lists may span lines.
  Values Flatten(const std::string& content) {
    Values values;
    std::vector<std::pair<size_t, std::string>> parents;
    std::istringstream stream(content);
    std::string raw, pending_key;
    while (std::getline(stream, raw)) {
      std::string line = StripComment(raw);
      std::string trimmed = Trim(line);
      if (trimmed.empty()) {
        continue;
      }
      if (!pending_key.empty()) {
        values[pending_key] += " " + trimmed;
        if (trimmed.find(']') != std::string::npos) {
          pending_key.clear();
        }
        continue;
      }
      if (trimmed[0] == '%' || trimmed.rfind("---", 0) == 0 || trimmed[0] == '-') {
        continue;
      }

      size_t indent = line.find_first_not_of(" \t");
      size_t colon = trimmed.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      while (!parents.empty() && parents.back().first >= indent) {
        parents.pop_back();
      }

      std::string key = Unquote(Trim(trimmed.substr(0, colon)));
      std::string value = Trim(trimmed.substr(colon + 1));
      if (value.rfind("!!", 0) == 0) {
        size_t space = value.find_first_of(" \t");
        value = space == std::string::npos ? "" : Trim(value.substr(space));
      }

      std::string full_key;
      for (const auto& parent : parents) {
        full_key += parent.second + ".";
      }
      full_key += key;

      if (value.empty()) {
        parents.push_back({indent, key});
        continue;
      }
      values[full_key] = value;
      if (value[0] == '[' && value.find(']') == std::string::npos) {
        pending_key = full_key;
      }
    }
    return values;
  }

  bool Has(const Values& values, const std::string& key) {
    return values.count(key) > 0;
  }

  std::string Text(const Values& values, const std::string& key, const std::string& fallback = "") {
    auto it = values.find(key);
    return it == values.end() ? fallback : Unquote(it->second);
  }

  double Number(const Values& values, const std::string& key) {
    std::string text = Text(values, key);
    std::istringstream stream(text);
    double value;
    if (text.empty() || !(stream >> value) || !(stream >> std::ws).eof()) {
      throw std::runtime_error("Invalid or missing number '" + key + "'");
    }
    return value;
  }

  std::optional<double> OptionalNumber(const Values& values, const std::string& key) {
    if (!Has(values, key)) {
      return {};
    }
    return Number(values, key);
  }

  std::vector<double> List(const Values& values, const std::string& key) {
    auto it = values.find(key);
    if (it == values.end()) {
      throw std::runtime_error("Missing list '" + key + "'");
    }
    std::string text = it->second;
    size_t open = text.find('['), close = text.rfind(']');
    if (open == std::string::npos || close == std::string::npos || close < open) {
      throw std::runtime_error("Invalid list '" + key + "'");
    }
    text = text.substr(open + 1, close - open - 1);
    std::replace(text.begin(), text.end(), ',', ' ');
    std::istringstream stream(text);
    std::vector<double> list;
    double v;
    while (stream >> v) {
      list.push_back(v);
    }
    if (!(stream >> std::ws).eof()) {
      throw std::runtime_error("Non-numeric values in list '" + key + "'");
    }
    return list;
  }

  std::optional<std::string> FirstKey(const Values& values, const std::vector<std::string>& keys) {
    for (const auto& key : keys) {
      if (Has(values, key)) {
        return key;
      }
    }
    return {};
  }

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
      Values values = Flatten(content);
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
      Values values = Flatten(content);
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
      std::ifstream file(path);
      if (!file.is_open()) {
        return false;
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      return IsSupported(buffer.str());
    }

    std::vector<ImportedCamera> ReadCameras(const std::string& path) {
      std::ifstream file(path);
      if (!file.is_open()) {
        throw std::runtime_error("Could not open " + path);
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      return ParseCameras(buffer.str());
    }
  }
}
