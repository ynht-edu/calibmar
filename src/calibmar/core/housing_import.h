#pragma once

#include "calibmar/core/camera_models.h"

#include <string>
#include <vector>

namespace calibmar {
  // Housing description yaml, all lengths in meters:
  //
  //   housing: FLATPORT        # or DOMEPORT
  //   normal: [0, 0, 1]        # flat: initial interface normal (optimized)
  //   distance: 0.01           # flat: initial camera center to inner surface (optimized)
  //   center: [0, 0, 0]        # dome: initial dome center offset (optimized)
  //   radius: 0.05             # dome: inner radius
  //   thickness: 0.01          # interface thickness
  //   glass: 1.49              # refractive index of the interface
  //   water: 1.339             # refractive index of the water
  //   air: 1.0                 # optional, refractive index inside the housing
  //
  // Also accepted: 'housing' with a raw 'parameters' list, or a calibmar calibration yaml
  // (non_svp_model, non_svp_parameters) to start from a previous housing calibration.
  namespace housing_import {
    struct ImportedHousing {
      HousingInterfaceType housing_interface;
      std::vector<double> params;
    };

    ImportedHousing ParseHousing(const std::string& content);
    ImportedHousing ReadHousing(const std::string& path);
  }
}
