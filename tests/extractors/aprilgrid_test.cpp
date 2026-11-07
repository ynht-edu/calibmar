#include <boost/test/unit_test.hpp>

#include "calibmar/extractors/aprilgrid.h"
#include "utils/memory_image_reader.h"

#include <opencv2/calib3d.hpp>

using namespace calibmar;

namespace {
  double PlanarHomographyRms(const Image& image, const std::map<uint32_t, Eigen::Vector3d>& points3D) {
    std::vector<cv::Point2f> board, pixels;
    for (const auto& [idx_2D, id_3D] : image.Correspondences()) {
      const Eigen::Vector3d& p3 = points3D.at(id_3D);
      const Eigen::Vector2d& p2 = image.Points2D()[idx_2D];
      board.emplace_back(p3.x(), p3.y());
      pixels.emplace_back(p2.x(), p2.y());
    }
    cv::Mat H = cv::findHomography(board, pixels, 0);
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(board, projected, H);
    double sum = 0;
    for (size_t i = 0; i < pixels.size(); i++) {
      cv::Point2f d = projected[i] - pixels[i];
      sum += d.dot(d);
    }
    return std::sqrt(sum / pixels.size());
  }

  double ExtractAndFit(const ArucoBoardFeatureExtractor::Options& options, size_t* num_points) {
    MemoryImageReader reader({MemoryImageReader::Images::AprilGridBoard});
    ArucoBoardFeatureExtractor extractor(options);
    Image image;
    Pixmap pixmap;
    reader.Next(image, pixmap);
    BOOST_REQUIRE(extractor.Extract(image, pixmap) == FeatureExtractor::Status::SUCCESS);
    *num_points = image.Points2D().size();
    return PlanarHomographyRms(image, extractor.Points3D());
  }
}

BOOST_AUTO_TEST_CASE(KalibrAprilGridYamlParsing) {
  const std::string yaml =
      "target_type: 'aprilgrid' #gridtype\n"
      "tagCols: 6               #number of apriltags\n"
      "tagRows: 7               #number of apriltags\n"
      "tagSize: 0.088           #size of apriltag, edge to edge [m]\n"
      "tagSpacing: 0.3          #ratio of space between tags to tagSize\n";

  aprilgrid::KalibrAprilGrid grid = aprilgrid::ParseKalibrTargetYaml(yaml);
  BOOST_TEST(grid.tag_cols == 6);
  BOOST_TEST(grid.tag_rows == 7);
  BOOST_TEST(grid.tag_size == 0.088);
  BOOST_TEST(grid.tag_spacing == 0.3);

  ArucoBoardFeatureExtractor::Options options = aprilgrid::ToArucoBoardOptions(grid);
  BOOST_TEST(options.marker_cols == 6);
  BOOST_TEST(options.marker_rows == 7);
  BOOST_TEST(options.marker_size == 0.088);
  BOOST_TEST(options.marker_spacing == 0.088 * 0.3);
  BOOST_TEST(aprilgrid::HasKalibrLayout(options));
}

BOOST_AUTO_TEST_CASE(KalibrYamlRejectsOtherTargets) {
  BOOST_CHECK_THROW(aprilgrid::ParseKalibrTargetYaml("target_type: 'checkerboard'\ntargetCols: 6\n"), std::runtime_error);
  BOOST_CHECK_THROW(aprilgrid::ParseKalibrTargetYaml("target_type: aprilgrid\ntagCols: 6\ntagRows: 6\ntagSize: 0.1\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(
      aprilgrid::ParseKalibrTargetYaml("target_type: aprilgrid\ntagCols: six\ntagRows: 6\ntagSize: 0.1\ntagSpacing: 0.3\n"),
      std::runtime_error);
}

BOOST_AUTO_TEST_CASE(KalibrAprilGridGeometryMatchesImage) {
  aprilgrid::KalibrAprilGrid grid;
  grid.tag_cols = 6;
  grid.tag_rows = 6;
  grid.tag_size = 1.0;
  grid.tag_spacing = 0.3;

  size_t num_points = 0;
  double rms = ExtractAndFit(aprilgrid::ToArucoBoardOptions(grid), &num_points);
  BOOST_TEST(num_points >= 4 * 30);
  BOOST_TEST(rms < 1.5);

  ArucoBoardFeatureExtractor::Options mirrored = aprilgrid::ToArucoBoardOptions(grid);
  mirrored.grid_origin = ArucoGridOrigin::TopLeft;
  BOOST_TEST(ExtractAndFit(mirrored, &num_points) > 10.0);
}
