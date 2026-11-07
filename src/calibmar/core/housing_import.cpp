#include "calibmar/core/housing_import.h"
#include "calibmar/core/simple_yaml.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace {
  using namespace calibmar;
  using namespace simple_yaml;

  std::string Lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
    return s;
  }

  HousingInterfaceType HousingFromName(const std::string& name) {
    for (const auto& [type, housing] : HousingInterface::HousingInterfaces()) {
      if (housing.model_name == name) {
        return type;
      }
    }
    throw std::runtime_error("Unknown housing '" + name + "', expected FLATPORT or DOMEPORT");
  }

  double Index(const Values& values, const std::string& key, std::optional<double> fallback = {}) {
    if (!Has(values, key)) {
      if (fallback) {
        return *fallback;
      }
      throw std::runtime_error("Housing yaml is missing '" + key + "'");
    }
    double value = Number(values, key);
    if (value < 1.0 || value > 3.0) {
      throw std::runtime_error("Housing '" + key + "' = " + Text(values, key) + " is not a refractive index (1.0 - 3.0)");
    }
    return value;
  }

  double Length(const Values& values, const std::string& key, std::optional<double> fallback = {}) {
    std::optional<double> value = OptionalNumber(values, key);
    if (!value) {
      if (fallback) {
        return *fallback;
      }
      throw std::runtime_error("Housing yaml is missing '" + key + "'");
    }
    if (*value < 0 || *value > 1.0) {
      throw std::runtime_error("Housing '" + key + "' = " + Text(values, key) + " looks wrong, lengths are in meters");
    }
    return *value;
  }

  std::vector<double> Vector3(const Values& values, const std::string& key, const std::vector<double>& fallback) {
    if (!Has(values, key)) {
      return fallback;
    }
    std::vector<double> v = List(values, key);
    if (v.size() != 3) {
      throw std::runtime_error("Housing '" + key + "' must have 3 values");
    }
    return v;
  }

  void CheckParams(HousingInterfaceType type, std::vector<double>& params) {
    size_t expected = HousingInterface::HousingInterfaces().at(type).num_params;
    if (params.size() != expected) {
      throw std::runtime_error("Housing parameters must have " + std::to_string(expected) + " values, got " +
                               std::to_string(params.size()));
    }
    if (type == HousingInterfaceType::DoubleLayerPlanarRefractive) {
      double norm = std::sqrt(params[0] * params[0] + params[1] * params[1] + params[2] * params[2]);
      if (norm < 1e-9) {
        throw std::runtime_error("Flat port normal must not be zero");
      }
      for (size_t i = 0; i < 3; i++) {
        params[i] /= norm;
      }
    }
  }
}

namespace calibmar {
  namespace housing_import {
    ImportedHousing ParseHousing(const std::string& content) {
      Values values = simple_yaml::Parse(content);
      ImportedHousing housing;

      if (Has(values, "non_svp_model") && Has(values, "non_svp_parameters")) {
        housing.housing_interface = HousingFromName(Text(values, "non_svp_model"));
        housing.params = List(values, "non_svp_parameters");
        CheckParams(housing.housing_interface, housing.params);
        return housing;
      }

      if (!Has(values, "housing")) {
        throw std::runtime_error("Housing yaml needs 'housing: FLATPORT' or 'housing: DOMEPORT'");
      }
      std::string name = Text(values, "housing");
      std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::toupper(c); });
      housing.housing_interface = HousingFromName(name);

      if (Has(values, "parameters")) {
        housing.params = List(values, "parameters");
        CheckParams(housing.housing_interface, housing.params);
        return housing;
      }

      double thickness = Length(values, "thickness");
      double na = Index(values, "air", 1.0);
      double ng = Index(values, "glass");
      double nw = Index(values, "water");

      if (housing.housing_interface == HousingInterfaceType::DoubleLayerPlanarRefractive) {
        housing.params = Vector3(values, "normal", {0, 0, 1});
        housing.params.push_back(Length(values, "distance", 0.01));
      }
      else {
        housing.params = Vector3(values, "center", {0, 0, 0});
        housing.params.push_back(Length(values, "radius"));
      }
      housing.params.insert(housing.params.end(), {thickness, na, ng, nw});
      CheckParams(housing.housing_interface, housing.params);
      return housing;
    }

    ImportedHousing ReadHousing(const std::string& path) {
      return ParseHousing(simple_yaml::ReadFile(path));
    }
  }
}
