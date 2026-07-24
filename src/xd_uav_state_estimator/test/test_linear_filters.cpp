#include <gtest/gtest.h>

#include <xd_uav_state_estimator/linear_filters.h>

namespace xd_uav_state_estimator {

TEST(LinearFilters, InitializesAllTranslationalStates) {
  LinearFilters filters(FilterParameters{});
  const Eigen::Vector3d position(1.0, -2.0, 3.0);
  const Eigen::Vector3d velocity(0.5, -0.25, 0.1);
  filters.initialize(position, velocity, ros::Time(10.0));

  EXPECT_TRUE(filters.initialized());
  EXPECT_TRUE(filters.position().isApprox(position));
  EXPECT_TRUE(filters.velocity().isApprox(velocity));
  EXPECT_TRUE(filters.acceleration().isZero());
  EXPECT_TRUE(filters.finite());
}

TEST(LinearFilters, ConstantVelocityPredictionMatchesModel) {
  LinearFilters filters(FilterParameters{});
  filters.initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, -2.0, 0.5), ros::Time(1.0));
  filters.predict(0.1, Eigen::Vector3d::Zero(), false, ros::Time(1.1));

  EXPECT_NEAR(filters.position().x(), 0.1, 1e-9);
  EXPECT_NEAR(filters.position().y(), -0.2, 1e-9);
  EXPECT_NEAR(filters.position().z(), 0.05, 1e-9);
  EXPECT_TRUE(filters.finite());
}

TEST(LinearFilters, AccelerationInputDrivesPredictionState) {
  FilterParameters parameters;
  parameters.input_coefficient = 10.0;
  LinearFilters filters(parameters);
  filters.initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                     ros::Time(1.0));

  const Eigen::Vector3d acceleration(1.0, -2.0, 0.5);
  filters.predict(0.1, acceleration, true, ros::Time(1.1));
  EXPECT_TRUE(filters.acceleration().isApprox(acceleration, 1e-12));

  filters.predict(0.1, acceleration, true, ros::Time(1.2));
  EXPECT_NEAR(filters.position().x(), 0.005, 1e-12);
  EXPECT_NEAR(filters.position().y(), -0.010, 1e-12);
  EXPECT_NEAR(filters.position().z(), 0.0025, 1e-12);
  EXPECT_NEAR(filters.velocity().x(), 0.1, 1e-12);
  EXPECT_NEAR(filters.velocity().y(), -0.2, 1e-12);
  EXPECT_NEAR(filters.velocity().z(), 0.05, 1e-12);
}

TEST(LinearFilters, RepeatedCorrectionsConverge) {
  LinearFilters filters(FilterParameters{});
  filters.initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ros::Time(1.0));
  const Eigen::Vector3d target_position(4.0, -3.0, 2.0);
  const Eigen::Vector3d target_velocity(0.4, -0.3, 0.2);

  for (int i = 1; i <= 100; ++i) {
    const ros::Time stamp(1.0 + 0.01 * i);
    filters.predict(0.01, Eigen::Vector3d::Zero(), false, stamp);
    filters.correctPosition(target_position, 0.01, 0.01, stamp);
    filters.correctVelocity(target_velocity, 0.01, 0.01, stamp);
  }

  EXPECT_NEAR(filters.position().x(), target_position.x(), 0.02);
  EXPECT_NEAR(filters.position().y(), target_position.y(), 0.02);
  EXPECT_NEAR(filters.position().z(), target_position.z(), 0.02);
  EXPECT_NEAR(filters.velocity().x(), target_velocity.x(), 0.02);
  EXPECT_NEAR(filters.velocity().y(), target_velocity.y(), 0.02);
  EXPECT_NEAR(filters.velocity().z(), target_velocity.z(), 0.02);
  EXPECT_TRUE(filters.finite());
}

TEST(LinearFilters, AltitudeAndLateralCorrectionsAreIndependent) {
  LinearFilters filters(FilterParameters{});
  filters.initialize(Eigen::Vector3d(1.0, 2.0, 3.0),
                     Eigen::Vector3d::Zero(), ros::Time(1.0));

  filters.correctLateralPosition(Eigen::Vector2d(10.0, 20.0), 1e-6,
                                 ros::Time(1.0));
  EXPECT_NEAR(filters.position().z(), 3.0, 1e-12);

  const Eigen::Vector2d lateral_before_altitude = filters.position().head<2>();
  filters.correctAltitudePosition(30.0, 1e-6, ros::Time(1.0));
  EXPECT_TRUE(filters.position().head<2>().isApprox(lateral_before_altitude,
                                                    1e-12));
}

TEST(LinearFilters, RestoresSavedSnapshot) {
  LinearFilters filters(FilterParameters{});
  const Eigen::Vector3d initial_position(1.0, 2.0, 3.0);
  filters.initialize(initial_position, Eigen::Vector3d::Zero(), ros::Time(1.0));
  const LinearFilters::Snapshot snapshot = filters.snapshot();

  filters.predict(0.5, Eigen::Vector3d(1.0, 0.0, 0.0), true,
                  ros::Time(1.5));
  filters.restore(snapshot);

  EXPECT_TRUE(filters.position().isApprox(initial_position, 1e-12));
  EXPECT_EQ(filters.stamp(), ros::Time(1.0));
  EXPECT_TRUE(filters.finite());
}

}  // 命名空间 xd_uav_state_estimator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
