#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/core/report.h"
#include "calibmar/extractors/aprilgrid.h"
#include "calibmar/readers/filesystem_reader.h"

#include <boost/program_options.hpp>
#include <clocale>
#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace calibmar;
namespace po = boost::program_options;

namespace {
  CameraModelType CameraModelFromName(const std::string& name) {
    for (const auto& [type, model] : CameraModel::CameraModels()) {
      if (model.model_name == name) {
        return type;
      }
    }
    throw std::runtime_error("Unknown camera model: " + name);
  }

  std::string CameraModelNames() {
    std::string names;
    for (const auto& [type, model] : CameraModel::CameraModels()) {
      names += (names.empty() ? "" : ", ") + model.model_name;
    }
    return names;
  }
}

int main(int argc, char* argv[]) {
  std::setlocale(LC_ALL, "C");

  std::string images_dir, target_file, camera_model = "OPENCV", output_file = "calibration.txt";
  int min_points = 8;

  po::options_description desc("Calibrate a camera from images of a Kalibr AprilGrid");
  desc.add_options()("help,h", "Show this help")(
      "images,i", po::value(&images_dir)->required(), "Directory with calibration images")(
      "target,t", po::value(&target_file)->required(), "Kalibr aprilgrid target.yaml")(
      "camera_model,m", po::value(&camera_model)->default_value(camera_model),
      ("Camera model: " + CameraModelNames()).c_str())(
      "output,o", po::value(&output_file)->default_value(output_file),
      "Report path, a .yaml with the same name is written next to it")(
      "min_points", po::value(&min_points)->default_value(min_points), "Skip images with fewer detected corners");

  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 0;
    }
    po::notify(vm);
  }
  catch (const std::exception& ex) {
    std::cerr << ex.what() << "\n\n" << desc << std::endl;
    return 1;
  }

  try {
    ArucoBoardFeatureExtractor::Options target_options =
        aprilgrid::ToArucoBoardOptions(aprilgrid::ReadKalibrTargetYaml(target_file));
    ArucoBoardFeatureExtractor extractor(target_options);

    FilesystemImageReader::Options reader_options;
    reader_options.image_directory = images_dir;
    FilesystemImageReader reader(reader_options);
    std::pair<int, int> image_size = {reader.ImagesWidth(), reader.ImagesHeight()};

    Calibration calibration;
    calibration.SetPoints3D(extractor.Points3D());
    calibration.SetCalibrationTargetInfo(report::GenerateCalibrationTargetInfo(target_options));

    std::cout << "Target: " << target_options.marker_cols << "x" << target_options.marker_rows
              << " tags, size " << target_options.marker_size << ", spacing " << target_options.marker_spacing << "\n"
              << "Image size: " << image_size.first << "x" << image_size.second << "\n\n";

    while (reader.HasNext()) {
      Image image;
      Pixmap pixmap;
      ImageReader::Status read_status = reader.Next(image, pixmap);
      std::string name = image.Name();
      if (read_status != ImageReader::Status::SUCCESS) {
        std::cout << "  skip  " << name << " (unreadable or size mismatch)\n";
        continue;
      }
      if (extractor.Extract(image, pixmap) != FeatureExtractor::Status::SUCCESS ||
          image.Points2D().size() < static_cast<size_t>(min_points)) {
        std::cout << "  skip  " << name << " (no grid)\n";
        continue;
      }
      calibration.AddImage(image);
      std::cout << "  ok    " << name << " (" << image.Points2D().size() << " corners)\n";
    }

    if (calibration.Images().size() < 3) {
      throw std::runtime_error("Need at least 3 images with a detected grid, got " +
                               std::to_string(calibration.Images().size()));
    }

    BasicCalibrator::Options calibrator_options;
    calibrator_options.camera_model = CameraModelFromName(camera_model);
    calibrator_options.image_size = image_size;
    BasicCalibrator calibrator(calibrator_options);
    calibrator.Calibrate(calibration);

    std::cout << "\nPer image RMS:\n";
    for (size_t i = 0; i < calibration.Images().size() && i < calibration.PerViewRms().size(); i++) {
      std::cout << "  " << std::fixed << std::setprecision(3) << calibration.PerViewRms()[i] << "  "
                << calibration.Images()[i].Name() << "\n";
    }
    std::cout.unsetf(std::ios::fixed);
    std::cout << "\n" << report::GenerateResultString(calibration) << std::endl;

    std::filesystem::path report_path(output_file);
    std::filesystem::path yaml_path = report_path;
    yaml_path.replace_extension("yaml");
    report::WriteCalibrationReport(report_path.string(), calibration);
    report::WriteCalibrationYaml(yaml_path.string(), calibration);
    std::cout << "\nWritten: " << report_path.string() << ", " << yaml_path.string() << std::endl;
  }
  catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
