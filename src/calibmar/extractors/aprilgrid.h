#pragma once

#include "calibmar/extractors/aruco_board_extractor.h"

#include <string>

namespace calibmar {
  namespace aprilgrid {
    // Kalibr AprilGrid: AprilTag 36h11, 2 bit border, tag 0 at bottom left, IDs ascending row-wise to the right.
    struct KalibrAprilGrid {
      int tag_cols = 6;
      int tag_rows = 6;
      double tag_size = 0.088;
      // ratio of the gap between tags to the tag size (Kalibr 'tagSpacing')
      double tag_spacing = 0.3;
    };

    ArucoBoardFeatureExtractor::Options ToArucoBoardOptions(const KalibrAprilGrid& grid);

    // Applies the Kalibr layout (dictionary, border, origin, direction) and keeps the board dimensions.
    void ApplyKalibrLayout(ArucoBoardFeatureExtractor::Options& options);

    bool HasKalibrLayout(const ArucoBoardFeatureExtractor::Options& options);

    KalibrAprilGrid ParseKalibrTargetYaml(const std::string& content);

    KalibrAprilGrid ReadKalibrTargetYaml(const std::string& path);
  }
}
