#include <boost/test/unit_test.hpp>

#include "calibmar/core/housing_import.h"

using namespace calibmar;

BOOST_TEST_DONT_PRINT_LOG_VALUE(HousingInterfaceType)

BOOST_AUTO_TEST_CASE(HousingYamlFlatPort) {
  housing_import::ImportedHousing housing = housing_import::ParseHousing(R"(# 10 mm acrylic flat port
housing: FLATPORT
distance: 0.02
thickness: 0.010
glass: 1.49
water: 1.339
)");
  BOOST_TEST(housing.housing_interface == HousingInterfaceType::DoubleLayerPlanarRefractive);
  std::vector<double> expected{0, 0, 1, 0.02, 0.01, 1.0, 1.49, 1.339};
  BOOST_TEST(housing.params == expected, boost::test_tools::per_element());

  housing = housing_import::ParseHousing("housing: flatport\nnormal: [0, 0, 2]\nthickness: 0.008\nglass: 1.52\nwater: 1.333\n");
  std::vector<double> expected_defaults{0, 0, 1, 0.01, 0.008, 1.0, 1.52, 1.333};
  BOOST_TEST(housing.params == expected_defaults, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(HousingYamlDomePort) {
  housing_import::ImportedHousing housing = housing_import::ParseHousing(R"(housing: DOMEPORT
center: [0.001, -0.002, 0.003]
radius: 0.05
thickness: 0.007
glass: 1.47
water: 1.34
)");
  BOOST_TEST(housing.housing_interface == HousingInterfaceType::DoubleLayerSphericalRefractive);
  std::vector<double> expected{0.001, -0.002, 0.003, 0.05, 0.007, 1.0, 1.47, 1.34};
  BOOST_TEST(housing.params == expected, boost::test_tools::per_element());
}

BOOST_AUTO_TEST_CASE(HousingYamlRawAndPreviousCalibration) {
  housing_import::ImportedHousing raw =
      housing_import::ParseHousing("housing: FLATPORT\nparameters: [0, 0, 1, 0.01, 0.01, 1, 1.49, 1.333]\n");
  BOOST_TEST(raw.params.size() == 8u);

  housing_import::ImportedHousing previous = housing_import::ParseHousing(R"(model: OPENCV
# fx, fy, cx, cy, k1, k2, p1, p2
parameters: [900, 905, 652.5, 478.5, -0.12, 0.05, 0.001, -0.0005]
non_svp_model: FLATPORT
# Nx, Ny, Nz, int_dist, int_thick, na, ng, nw (Note that [Nx, Ny, Nz] must be unit vector)
non_svp_parameters: [0.02089748307, -0.01463284637, 0.9996745345, 0.027342224, 0.01, 1, 1.49, 1.339]
width: 1280
height: 960
)");
  BOOST_TEST(previous.housing_interface == HousingInterfaceType::DoubleLayerPlanarRefractive);
  BOOST_TEST(previous.params[3] == 0.027342224, boost::test_tools::tolerance(1e-12));
}

BOOST_AUTO_TEST_CASE(HousingYamlErrors) {
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: FLATPORT\nthickness: 10\nglass: 1.49\nwater: 1.339\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: FLATPORT\nthickness: 0.01\nglass: acrylic\nwater: 1.339\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: FLATPORT\nthickness: 0.01\nglass: 1.49\nwater: 0.5\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: DOMEPORT\nthickness: 0.01\nglass: 1.49\nwater: 1.339\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: CYLINDER\nthickness: 0.01\nglass: 1.49\nwater: 1.339\n"),
                    std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("thickness: 0.01\nglass: 1.49\nwater: 1.339\n"), std::runtime_error);
  BOOST_CHECK_THROW(housing_import::ParseHousing("housing: FLATPORT\nparameters: [0, 0, 1, 0.01]\n"), std::runtime_error);
}
