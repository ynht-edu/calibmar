#pragma once

#include "calibmar/core/camera_import.h"

#include <QtWidgets>
#include <optional>

namespace calibmar {
  // Returns the selected camera of a Kalibr/ROS/OpenCV/ORB-SLAM3 file, asking the user if it contains more than one.
  inline std::optional<camera_import::ImportedCamera> SelectImportedCamera(QWidget* parent, const std::string& path,
                                                                           size_t default_index = 0) {
    std::vector<camera_import::ImportedCamera> cameras;
    try {
      cameras = camera_import::ReadCameras(path);
    }
    catch (const std::exception& ex) {
      QMessageBox::warning(parent, "Import", QString::fromStdString(ex.what()));
      return {};
    }

    size_t index = std::min(default_index, cameras.size() - 1);
    if (cameras.size() > 1) {
      QStringList names;
      for (const auto& camera : cameras) {
        names << QString::fromStdString(camera.name);
      }
      bool ok = false;
      QString name = QInputDialog::getItem(parent, "Import", QString::fromStdString(cameras[0].format + " camera"), names,
                                           static_cast<int>(index), false, &ok);
      if (!ok) {
        return {};
      }
      index = names.indexOf(name);
    }

    return cameras[index];
  }
}
