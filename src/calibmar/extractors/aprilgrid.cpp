#include "calibmar/extractors/aprilgrid.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
  std::string Trim(const std::string& s) {
    auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c); });
    auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c); }).base();
    return begin < end ? std::string(begin, end) : std::string();
  }

  std::string Unquote(const std::string& s) {
    if (s.size() >= 2 && (s.front() == '\'' || s.front() == '"') && s.back() == s.front()) {
      return s.substr(1, s.size() - 2);
    }
    return s;
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
      else if (c == '#') {
        return line.substr(0, i);
      }
    }
    return line;
  }

  const std::string& Require(const std::map<std::string, std::string>& values, const std::string& key) {
    auto it = values.find(key);
    if (it == values.end()) {
      throw std::runtime_error("Kalibr target yaml is missing '" + key + "'");
    }
    return it->second;
  }

  template <typename T>
  T ParseNumber(const std::map<std::string, std::string>& values, const std::string& key) {
    const std::string& text = Require(values, key);
    std::istringstream stream(text);
    T value;
    if (!(stream >> value) || !(stream >> std::ws).eof()) {
      throw std::runtime_error("Kalibr target yaml has invalid value for '" + key + "': " + text);
    }
    return value;
  }
}

namespace calibmar {
  namespace aprilgrid {
    ArucoBoardFeatureExtractor::Options ToArucoBoardOptions(const KalibrAprilGrid& grid) {
      ArucoBoardFeatureExtractor::Options options;
      options.marker_cols = grid.tag_cols;
      options.marker_rows = grid.tag_rows;
      options.marker_size = grid.tag_size;
      options.marker_spacing = grid.tag_size * grid.tag_spacing;
      options.start_id = 0;
      ApplyKalibrLayout(options);
      return options;
    }

    void ApplyKalibrLayout(ArucoBoardFeatureExtractor::Options& options) {
      options.aruco_type = ArucoMarkerTypes::DICT_APRILTAG_36h11;
      options.border_bits = 2;
      options.grid_origin = ArucoGridOrigin::BottomLeft;
      options.grid_direction = ArucoGridDirection::Horizontal;
    }

    bool HasKalibrLayout(const ArucoBoardFeatureExtractor::Options& options) {
      return options.aruco_type == ArucoMarkerTypes::DICT_APRILTAG_36h11 && options.border_bits == 2 &&
             options.grid_origin == ArucoGridOrigin::BottomLeft && options.grid_direction == ArucoGridDirection::Horizontal;
    }

    KalibrAprilGrid ParseKalibrTargetYaml(const std::string& content) {
      std::map<std::string, std::string> values;
      std::istringstream stream(content);
      std::string line;
      while (std::getline(stream, line)) {
        line = Trim(StripComment(line));
        size_t colon = line.find(':');
        if (line.empty() || colon == std::string::npos) {
          continue;
        }
        values[Trim(line.substr(0, colon))] = Unquote(Trim(line.substr(colon + 1)));
      }

      std::string type = Require(values, "target_type");
      std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::tolower(c); });
      if (type != "aprilgrid") {
        throw std::runtime_error("Kalibr target type '" + type + "' is not an aprilgrid");
      }

      KalibrAprilGrid grid;
      grid.tag_cols = ParseNumber<int>(values, "tagCols");
      grid.tag_rows = ParseNumber<int>(values, "tagRows");
      grid.tag_size = ParseNumber<double>(values, "tagSize");
      grid.tag_spacing = ParseNumber<double>(values, "tagSpacing");

      if (grid.tag_cols < 1 || grid.tag_rows < 1 || grid.tag_size <= 0 || grid.tag_spacing <= 0) {
        throw std::runtime_error("Kalibr target yaml has non-positive grid dimensions");
      }

      return grid;
    }

    KalibrAprilGrid ReadKalibrTargetYaml(const std::string& path) {
      std::ifstream file(path);
      if (!file.is_open()) {
        throw std::runtime_error("Could not open Kalibr target yaml: " + path);
      }
      std::stringstream buffer;
      buffer << file.rdbuf();
      return ParseKalibrTargetYaml(buffer.str());
    }
  }
}
