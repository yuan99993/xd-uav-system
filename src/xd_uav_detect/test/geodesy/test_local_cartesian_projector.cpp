#include <limits>

#include <gtest/gtest.h>

#include <xd_uav_detect/geodesy/local_cartesian_projector.hpp>

namespace {

geographic_msgs::GeoPoint origin() {
  geographic_msgs::GeoPoint point;
  point.latitude = 47.397743;
  point.longitude = 8.545594;
  point.altitude = 123.4;
  return point;
}

TEST(LocalCartesianProjector, OriginAndCardinalDisplacements) {
  xd_uav_detect::LocalCartesianProjector projector;
  ASSERT_TRUE(projector.reset(origin()));

  geographic_msgs::GeoPoint result;
  ASSERT_TRUE(projector.reverse(Eigen::Vector3d::Zero(), &result));
  EXPECT_NEAR(result.latitude, 47.397743000000006, 1e-11);
  EXPECT_NEAR(result.longitude, 8.545593999999998, 1e-11);
  EXPECT_NEAR(result.altitude, 123.4000000006, 1e-6);

  ASSERT_TRUE(projector.reverse(Eigen::Vector3d(10.0, 0.0, 0.0), &result));
  EXPECT_NEAR(result.latitude, 47.397742999923459, 1e-11);
  EXPECT_NEAR(result.longitude, 8.545726465771809, 1e-11);
  EXPECT_NEAR(result.altitude, 123.4000078268, 1e-6);

  ASSERT_TRUE(projector.reverse(Eigen::Vector3d(0.0, 10.0, 0.0), &result));
  EXPECT_NEAR(result.latitude, 47.397832943627087, 1e-11);
  EXPECT_NEAR(result.longitude, 8.545593999999996, 1e-11);

  ASSERT_TRUE(projector.reverse(Eigen::Vector3d(0.0, 0.0, 10.0), &result));
  EXPECT_NEAR(result.altitude, 133.4000000004, 1e-6);
}

TEST(LocalCartesianProjector, NegativeDisplacementAndNonzeroAltitude) {
  xd_uav_detect::LocalCartesianProjector projector;
  ASSERT_TRUE(projector.reset(origin()));
  geographic_msgs::GeoPoint result;
  ASSERT_TRUE(projector.reverse(Eigen::Vector3d(-10.0, -20.0, -5.0),
                                &result));
  EXPECT_NEAR(result.latitude, 47.397563112523862, 1e-11);
  EXPECT_NEAR(result.longitude, 8.545461534575384, 1e-11);
  EXPECT_NEAR(result.altitude, 118.4000392225, 1e-6);
}

TEST(LocalCartesianProjector, RejectsInvalidInputs) {
  xd_uav_detect::LocalCartesianProjector projector;
  auto invalid_origin = origin();
  invalid_origin.latitude = 91.0;
  EXPECT_FALSE(projector.reset(invalid_origin));
  EXPECT_FALSE(projector.initialized());

  ASSERT_TRUE(projector.reset(origin()));
  geographic_msgs::GeoPoint result;
  EXPECT_FALSE(projector.reverse(
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      &result));
  EXPECT_FALSE(projector.reverse(Eigen::Vector3d::Zero(), nullptr));
}

TEST(LocalCartesianProjector, GeographicLibRoundTrip) {
  const auto datum = origin();
  xd_uav_detect::LocalCartesianProjector projector;
  ASSERT_TRUE(projector.reset(datum));
  GeographicLib::LocalCartesian reference(
      datum.latitude, datum.longitude, datum.altitude);
  double east = 0.0;
  double north = 0.0;
  double up = 0.0;
  reference.Forward(47.5, 8.7, 456.0, east, north, up);

  geographic_msgs::GeoPoint result;
  ASSERT_TRUE(projector.reverse(Eigen::Vector3d(east, north, up), &result));
  EXPECT_NEAR(result.latitude, 47.5, 1e-11);
  EXPECT_NEAR(result.longitude, 8.7, 1e-11);
  EXPECT_NEAR(result.altitude, 456.0, 1e-6);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
