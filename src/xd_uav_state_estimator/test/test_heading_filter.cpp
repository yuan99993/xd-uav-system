#include <gtest/gtest.h>

#include <xd_uav_state_estimator/heading_filter.h>

#include <cmath>

namespace xd_uav_state_estimator {

TEST(HeadingFilter, PredictsWithAngularRate) {
  HeadingFilter filter(HeadingFilterParameters{});
  filter.initialize(0.0, 1.0, ros::Time(10.0));
  filter.predict(ros::Time(10.5));

  EXPECT_NEAR(filter.heading(), 0.5, 1e-9);
  EXPECT_NEAR(filter.rate(), 1.0, 1e-9);
  EXPECT_TRUE(filter.finite());
}

TEST(HeadingFilter, CorrectsAcrossAngleBoundary) {
  HeadingFilterParameters parameters;
  parameters.initial_heading_covariance = 1.0;
  HeadingFilter filter(parameters);
  filter.initialize(3.13, 0.0, ros::Time(10.0));
  filter.correctHeading(-3.13, 0.001, ros::Time(10.0));

  EXPECT_LT(std::abs(HeadingFilter::wrapAngle(filter.heading() + 3.13)), 0.03);
  EXPECT_TRUE(filter.finite());
}

TEST(HeadingFilter, WrapsAnglesToCanonicalRange) {
  EXPECT_NEAR(HeadingFilter::wrapAngle(3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(HeadingFilter::wrapAngle(-3.0 * M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(HeadingFilter::wrapAngle(0.25), 0.25, 1e-12);
}

}  // 命名空间 xd_uav_state_estimator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
