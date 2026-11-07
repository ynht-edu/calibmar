#include <boost/test/unit_test.hpp>

#include "calibmar/core/camera_import.h"

using namespace calibmar;
using camera_import::ImportedCamera;

BOOST_TEST_DONT_PRINT_LOG_VALUE(CameraModelType)

namespace {
  ImportedCamera ParseSingle(const std::string& yaml) {
    BOOST_TEST(camera_import::IsSupported(yaml));
    std::vector<ImportedCamera> cameras = camera_import::ParseCameras(yaml);
    BOOST_REQUIRE(cameras.size() == 1);
    return cameras[0];
  }
}

BOOST_AUTO_TEST_CASE(ImportKalibrStereoRadtan) {
  const std::string yaml = R"(cam0:
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
  distortion_model: radtan
  intrinsics: [458.654, 457.296, 367.215, 248.375]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
cam1:
  T_cn_cnm1:
  - [0.9999972564779, 0.002312067192, 0.000376008102, -0.110073808127]
  - [-0.002317135723, 0.999898048506, 0.014089835846, 0.000399121547]
  - [-0.000343393121, -0.014090668452, 0.999900662637, -0.000853702503]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [0]
  camera_model: pinhole
  distortion_coeffs: [-0.28368365, 0.07451284,
    -0.00010473, -3.555907e-05]
  distortion_model: radtan
  intrinsics: [457.587, 456.134, 379.999, 255.238]
  resolution: [752, 480]
  rostopic: /cam1/image_raw
)";

  BOOST_TEST(camera_import::IsSupported(yaml));
  std::vector<ImportedCamera> cameras = camera_import::ParseCameras(yaml);
  BOOST_REQUIRE(cameras.size() == 2);

  BOOST_TEST(cameras[0].name == "cam0");
  BOOST_TEST(cameras[0].format == "Kalibr camchain");
  BOOST_TEST(cameras[0].camera_model == CameraModelType::OpenCVCameraModel);
  std::vector<double> expected0{458.654, 457.296, 367.715, 248.875, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  BOOST_TEST(cameras[0].params == expected0, boost::test_tools::per_element());
  BOOST_TEST(cameras[0].resolution.first == 752);
  BOOST_TEST(cameras[0].resolution.second == 480);

  BOOST_TEST(cameras[1].name == "cam1");
  std::vector<double> expected1{457.587, 456.134, 380.499, 255.738, -0.28368365, 0.07451284, -0.00010473, -3.555907e-05};
  BOOST_TEST(cameras[1].params == expected1, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(ImportKalibrImuCamEquidistant) {
  ImportedCamera camera = ParseSingle(R"(cam0:
  T_cam_imu:
  - [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975]
  - [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768]
  - [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.0034, -0.0021, 0.0011, -0.0004]
  distortion_model: equidistant
  intrinsics: [190.97, 190.97, 254.93, 256.89]
  resolution: [512, 512]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.00327
)");
  BOOST_TEST(camera.camera_model == CameraModelType::OpenCVFisheyeCameraModel);
  std::vector<double> expected{190.97, 190.97, 255.43, 257.39, 0.0034, -0.0021, 0.0011, -0.0004};
  BOOST_TEST(camera.params == expected, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(ImportRosCameraInfo) {
  ImportedCamera camera = ParseSingle(R"(image_width: 640
image_height: 480
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [430.21554, 0., 306.6913, 0., 430.53093, 227.22281, 0., 0., 1.]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.337032, 0.093964, 0.000000, 0.000356, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1., 0., 0., 0., 1., 0., 0., 0., 1.]
projection_matrix:
  rows: 3
  cols: 4
  data: [300.0, 0., 310.0, 0., 0., 350.0, 220.0, 0., 0., 0., 1., 0.]
)");
  BOOST_TEST(camera.format == "ROS camera_info");
  BOOST_TEST(camera.name == "narrow_stereo");
  BOOST_TEST(camera.camera_model == CameraModelType::OpenCVCameraModel);
  std::vector<double> expected{430.21554, 430.53093, 307.1913, 227.72281, -0.337032, 0.093964, 0.0, 0.000356};
  BOOST_TEST(camera.params == expected, boost::test_tools::per_element());
  BOOST_TEST(camera.resolution.first == 640);

  ImportedCamera rational = ParseSingle(R"(image_width: 640
image_height: 480
camera_matrix:
  data: [500, 0, 320, 0, 501, 240, 0, 0, 1]
distortion_model: rational_polynomial
distortion_coefficients:
  data: [0.1, -0.2, 0.001, 0.002, 0.03, 0.4, -0.05, 0.06]
)");
  BOOST_TEST(rational.camera_model == CameraModelType::FullOpenCVCameraModel);
  BOOST_TEST(rational.params.size() == 12u);
  BOOST_TEST(rational.params[11] == 0.06);

  ImportedCamera fisheye = ParseSingle(R"(camera_matrix:
  data: [300, 0, 320, 0, 300, 240, 0, 0, 1]
distortion_model: equidistant
distortion_coefficients:
  data: [0.01, -0.02, 0.003, -0.0004]
)");
  BOOST_TEST(fisheye.camera_model == CameraModelType::OpenCVFisheyeCameraModel);
}

BOOST_AUTO_TEST_CASE(ImportOpenCVFileStorage) {
  ImportedCamera camera = ParseSingle(R"(%YAML:1.0
---
calibration_time: "Sat 07 Nov 2026 06:00:00 PM WIB"
nr_of_frames: 25
image_width: 640
image_height: 480
board_width: 9
board_height: 6
square_size: 50.
flags: 2
fisheye_model: 0
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 6.5746697944293521e+02, 0., 3.1950000000000000e+02, 0.,
       6.5746697944293521e+02, 2.3950000000000000e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 5
   cols: 1
   dt: d
   data: [ -4.1802327176423804e-01, 5.0715244063187526e-01, 0., 0.,
       -5.7843597214487474e-01 ]
avg_reprojection_error: 3.9e-01
)");
  BOOST_TEST(camera.format == "OpenCV");
  BOOST_TEST(camera.camera_model == CameraModelType::FullOpenCVCameraModel);
  std::vector<double> expected{657.46697944293521, 657.46697944293521, 320.0, 240.0, -0.41802327176423804,
                               0.50715244063187526, 0.0, 0.0, -0.57843597214487474, 0.0, 0.0, 0.0};
  BOOST_TEST(camera.params == expected, boost::test_tools::per_element());
  BOOST_TEST(camera.resolution.second == 480);
}

BOOST_AUTO_TEST_CASE(ImportOrbSlam3) {
  ImportedCamera euroc = ParseSingle(R"(%YAML:1.0
Camera.type: "PinHole"
Camera.fx: 458.654
Camera.fy: 457.296
Camera.cx: 367.215
Camera.cy: 248.375
Camera.k1: -0.28340811
Camera.k2: 0.07395907
Camera.p1: 0.00019359
Camera.p2: 1.76187114e-05
Camera.width: 752
Camera.height: 480
Camera.fps: 20.0
ORBextractor.nFeatures: 1000
)");
  BOOST_TEST(euroc.format == "ORB-SLAM3");
  BOOST_TEST(euroc.camera_model == CameraModelType::OpenCVCameraModel);
  std::vector<double> expected{458.654, 457.296, 367.715, 248.875, -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  BOOST_TEST(euroc.params == expected, boost::test_tools::per_element());
  BOOST_TEST(euroc.resolution.first == 752);

  const std::string tumvi = R"(%YAML:1.0
File.version: "1.0"
Camera.type: "KannalaBrandt8"
Camera1.fx: 190.978477
Camera1.fy: 190.973307
Camera1.cx: 254.931706
Camera1.cy: 256.897442
Camera1.k1: 0.0034823894022493434
Camera1.k2: 0.0007150348452162257
Camera1.k3: -0.0020532361418706202
Camera1.k4: 0.00020293673591811182
Camera2.fx: 190.442369
Camera2.fy: 190.434438
Camera2.cx: 252.597594
Camera2.cy: 254.917910
Camera2.k1: 0.0034003170790442797
Camera2.k2: 0.001766278153469831
Camera2.k3: -0.00266312569781606
Camera2.k4: 0.0003299517423931039
Stereo.T_c1_c2: !!opencv-matrix
  rows: 4
  cols: 4
  dt: f
  data: [0.999997256477797, -0.002317135723275, -0.000343393120620, 0.110074137800478,
         0.002312067192432, 0.999898048507103, -0.014090668452683, -0.000156612054392,
         0.000376008102320, 0.014089835846691, 0.999900662638081, 0.000889382785432,
         0, 0, 0, 1.000000000000000]
Camera.width: 512
Camera.height: 512
)";
  BOOST_TEST(camera_import::IsSupported(tumvi));
  std::vector<ImportedCamera> cameras = camera_import::ParseCameras(tumvi);
  BOOST_REQUIRE(cameras.size() == 2);
  BOOST_TEST(cameras[1].name == "Camera2");
  BOOST_TEST(cameras[1].camera_model == CameraModelType::OpenCVFisheyeCameraModel);
  BOOST_TEST(cameras[1].params[2] == 253.097594);
  BOOST_TEST(cameras[1].params[7] == 0.0003299517423931039);
  BOOST_TEST(cameras[1].resolution.first == 512);
}

BOOST_AUTO_TEST_CASE(ImportRejectsUnsupported) {
  BOOST_CHECK_THROW(camera_import::ParseCameras(R"(cam0:
  camera_model: omni
  distortion_model: radtan
  distortion_coeffs: [0, 0, 0, 0]
  intrinsics: [0.8, 400, 400, 320, 240]
)"),
                    std::runtime_error);

  BOOST_CHECK_THROW(camera_import::ParseCameras(R"(cam0:
  camera_model: pinhole
  distortion_model: fov
  distortion_coeffs: [0.9]
  intrinsics: [400, 400, 320, 240]
)"),
                    std::runtime_error);

  BOOST_CHECK_THROW(camera_import::ParseCameras(R"(camera_matrix:
  data: [500, 0, 320, 0, 501, 240, 0, 0, 1]
distortion_coefficients:
  data: [0.1, -0.2, 0.001, 0.002, 0.03, 0, 0, 0, 0.01, 0.02, 0, 0]
)"),
                    std::runtime_error);

  const std::string calibmar_yaml = "model: OPENCV\nparameters: [1, 2, 3, 4, 0, 0, 0, 0]\nwidth: 640\nheight: 480\n";
  BOOST_TEST(!camera_import::IsSupported(calibmar_yaml));
}
