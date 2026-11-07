#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace calibmar {
  // Minimal reader for flat calibration yaml files (Kalibr, ROS, OpenCV FileStorage, ORB-SLAM3, calibmar).
  // Nested mappings are flattened to dotted keys, block sequences are skipped, flow lists may span lines.
  namespace simple_yaml {
    using Values = std::map<std::string, std::string>;

    Values Parse(const std::string& content);
    std::string ReadFile(const std::string& path);

    bool Has(const Values& values, const std::string& key);
    std::string Text(const Values& values, const std::string& key, const std::string& fallback = "");
    double Number(const Values& values, const std::string& key);
    std::optional<double> OptionalNumber(const Values& values, const std::string& key);
    std::vector<double> List(const Values& values, const std::string& key);
    std::optional<std::string> FirstKey(const Values& values, const std::vector<std::string>& keys);
  }
}
