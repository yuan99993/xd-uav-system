#include <gtest/gtest.h>

#include <xd_uav_state_estimator/imu_acceleration_processor.h>

namespace xd_uav_state_estimator {

TEST(ImuAccelerationProcessor, RemovesGravity) {
  ImuAccelerationParameters parameters;
  parameters.estimate_bias = false;
  ImuAccelerationProcessor processor(parameters);

  Eigen::Vector3d acceleration;
  ASSERT_TRUE(processor.update(Eigen::Vector3d(0.0, 0.0, 9.80665),
                               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                               &acceleration));
  EXPECT_TRUE(acceleration.isZero(1e-12));
}

TEST(ImuAccelerationProcessor, CalibratesOnlyFromConsecutiveStationarySamples) {
  ImuAccelerationParameters parameters;
  parameters.bias_samples = 3;
  parameters.initial_bias.setZero();
  ImuAccelerationProcessor processor(parameters);

  const Eigen::Vector3d biased_specific_force(0.10, -0.20, 9.85665);
  Eigen::Vector3d acceleration;
  EXPECT_FALSE(processor.update(biased_specific_force, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), &acceleration));
  EXPECT_EQ(processor.biasSampleCount(), 1U);

  EXPECT_FALSE(processor.update(biased_specific_force, Eigen::Vector3d(1.0, 0.0, 0.0),
                                Eigen::Vector3d::Zero(), &acceleration));
  EXPECT_EQ(processor.biasSampleCount(), 0U);

  EXPECT_FALSE(processor.update(biased_specific_force, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), &acceleration));
  EXPECT_FALSE(processor.update(biased_specific_force, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero(), &acceleration));
  ASSERT_TRUE(processor.update(biased_specific_force, Eigen::Vector3d::Zero(),
                               Eigen::Vector3d::Zero(), &acceleration));

  EXPECT_TRUE(processor.biasReady());
  EXPECT_TRUE(processor.bias().isApprox(Eigen::Vector3d(0.10, -0.20, 0.05), 1e-12));
  EXPECT_TRUE(acceleration.isZero(1e-12));
}

TEST(ImuAccelerationProcessor, RejectsUnreasonableAcceleration) {
  ImuAccelerationParameters parameters;
  parameters.estimate_bias = false;
  parameters.acceleration_limit = 5.0;
  ImuAccelerationProcessor processor(parameters);

  Eigen::Vector3d acceleration;
  EXPECT_FALSE(processor.update(Eigen::Vector3d(100.0, 0.0, 9.80665),
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                &acceleration));
}

}  // 命名空间 xd_uav_state_estimator

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
