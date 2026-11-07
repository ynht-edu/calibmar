#include "calibmar/calibrators/basic_calibrator.h"
#include "calibmar/calibrators/housing_calibrator.h"
#include "calibmar/core/camera_import.h"
#include "calibmar/core/report.h"
#include "calibmar/extractors/aprilgrid.h"
#include "calibmar/readers/filesystem_reader.h"

#include <boost/program_options.hpp>
#include <algorithm>
#include <clocale>
#include <optional>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

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

  HousingInterfaceType HousingFromName(const std::string& name) {
    for (const auto& [type, housing] : HousingInterface::HousingInterfaces()) {
      if (housing.model_name == name) {
        return type;
      }
    }
    throw std::runtime_error("Unknown housing model: " + name);
  }

  std::vector<double> ParseParams(const std::string& text) {
    std::string values = text;
    std::replace(values.begin(), values.end(), ',', ' ');
    std::istringstream stream(values);
    std::vector<double> params;
    double v;
    while (stream >> v) {
      params.push_back(v);
    }
    if (!(stream >> std::ws).eof()) {
      throw std::runtime_error("Invalid parameter list: " + text);
    }
    return params;
  }

  camera_import::ImportedCamera SelectCamera(const std::string& path, const std::string& selection) {
    std::vector<camera_import::ImportedCamera> cameras = camera_import::ReadCameras(path);
    if (selection.empty()) {
      return cameras.front();
    }
    std::string names;
    for (size_t i = 0; i < cameras.size(); i++) {
      if (cameras[i].name == selection || std::to_string(i) == selection) {
        return cameras[i];
      }
      names += (names.empty() ? "" : ", ") + cameras[i].name;
    }
    throw std::runtime_error("Camera '" + selection + "' not found in " + path + ", available: " + names);
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
  std::string intrinsics_file, camera_selection, housing, housing_params;
  int min_points = 8;

  po::options_description desc("Calibrate a camera, optionally with an underwater housing, from images of a Kalibr AprilGrid");
  desc.add_options()("help,h", "Show this help")(
      "images,i", po::value(&images_dir)->required(), "Directory with calibration images")(
      "target,t", po::value(&target_file)->required(), "Kalibr aprilgrid target.yaml")(
      "camera_model,m", po::value(&camera_model)->default_value(camera_model),
      ("Camera model: " + CameraModelNames()).c_str())(
      "output,o", po::value(&output_file)->default_value(output_file),
      "Report path, a .yaml with the same name is written next to it")(
      "min_points", po::value(&min_points)->default_value(min_points), "Skip images with fewer detected corners")(
      "intrinsics,c", po::value(&intrinsics_file),
      "In-air intrinsics: Kalibr camchain(-imucam).yaml, ROS camera_info, OpenCV or ORB-SLAM3 yaml. Overrides "
      "--camera_model. Used as initial guess, or kept fixed with --housing")(
      "camchain", po::value(&intrinsics_file), "Same as --intrinsics")(
      "cam", po::value(&camera_selection), "Camera in the intrinsics file by name (cam1, Camera2) or index. Default: first")(
      "housing", po::value(&housing), "Housing model, FLATPORT or DOMEPORT. Requires --intrinsics")(
      "housing_params", po::value(&housing_params),
      "Initial housing parameters, comma separated.\n"
      "FLATPORT: Nx, Ny, Nz, int_dist, int_thick, na, ng, nw\n"
      "DOMEPORT: Cx, Cy, Cz, int_radius, int_thick, na, ng, nw");

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

    std::optional<camera_import::ImportedCamera> camera;
    if (!intrinsics_file.empty()) {
      camera = SelectCamera(intrinsics_file, camera_selection);
      if (camera->resolution.first > 0 && camera->resolution != image_size) {
        throw std::runtime_error("Intrinsics resolution " + std::to_string(camera->resolution.first) + "x" +
                                 std::to_string(camera->resolution.second) + " does not match images " +
                                 std::to_string(image_size.first) + "x" + std::to_string(image_size.second));
      }
      std::cout << "In-air camera: " << camera->format << " " << camera->name << " ("
                << CameraModel::CameraModels().at(camera->camera_model).model_name << ")\n";
    }
    else if (!housing.empty()) {
      throw std::runtime_error("--housing needs in-air intrinsics, pass --intrinsics");
    }

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

    std::unique_ptr<Calibrator> calibrator;
    if (!housing.empty()) {
      if (!camera.has_value()) {
        throw std::runtime_error("--housing needs in-air intrinsics, pass --intrinsics");
      }
      HousingCalibrator::Options calibrator_options;
      calibrator_options.camera_model = camera->camera_model;
      calibrator_options.camera_params = camera->params;
      calibrator_options.housing_interface = HousingFromName(housing);
      calibrator_options.initial_housing_params = ParseParams(housing_params);
      calibrator_options.estimate_initial_dome_offset = false;
      calibrator_options.image_size = image_size;
      calibrator = std::make_unique<HousingCalibrator>(calibrator_options);
    }
    else {
      BasicCalibrator::Options calibrator_options;
      calibrator_options.image_size = image_size;
      if (camera.has_value()) {
        calibration.SetCamera(CameraModel::InitCamera(camera->camera_model, image_size, camera->params));
        calibrator_options.use_intrinsics_guess = true;
      }
      else {
        calibrator_options.camera_model = CameraModelFromName(camera_model);
      }
      calibrator = std::make_unique<BasicCalibrator>(calibrator_options);
    }
    calibrator->Calibrate(calibration);

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
    if (report_path.has_parent_path()) {
      std::filesystem::create_directories(report_path.parent_path());
    }
    report::WriteCalibrationReport(report_path.string(), calibration);
    report::WriteCalibrationYaml(yaml_path.string(), calibration);
    if (!std::filesystem::exists(report_path) || !std::filesystem::exists(yaml_path)) {
      throw std::runtime_error("Could not write " + report_path.string() + " / " + yaml_path.string());
    }
    std::cout << "\nWritten: " << report_path.string() << ", " << yaml_path.string() << std::endl;
  }
  catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  return 0;
}
