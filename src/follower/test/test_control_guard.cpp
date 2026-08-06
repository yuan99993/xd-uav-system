#include <gtest/gtest.h>

#include <follower/control_guard.hpp>

TEST(ControlGuard, AllowsCommandOnlyWithoutLeaseInCompatibilityMode) {
  follower::ControlGuardConfig config;
  config.require_lease = false;
  config.output_backend = follower::OutputBackend::kCommandOnly;
  follower::ControlGuard guard(config);
  EXPECT_TRUE(guard.authorized(10.0));
  EXPECT_EQ("command_only",
            std::string(follower::outputBackendName(guard.outputBackend())));
}

TEST(ControlGuard, EnforcesBackendOwnerAndExpiry) {
  follower::ControlGuardConfig config;
  config.require_lease = true;
  config.output_backend = follower::OutputBackend::kMavrosBody;
  follower::ControlGuard guard(config);
  EXPECT_FALSE(guard.authorized(1.0));
  EXPECT_FALSE(guard.acquire("ui", "mrs_velocity", 1.0, 1.0).success);
  EXPECT_TRUE(guard.acquire("ui", "mavros_body", 1.0, 1.0).success);
  EXPECT_TRUE(guard.authorized(1.5));
  EXPECT_FALSE(guard.acquire("automation", "mavros_body", 1.0, 1.5).success);
  EXPECT_FALSE(guard.authorized(2.0));
  EXPECT_TRUE(guard.leaseExpired(2.0));
}

TEST(ControlGuard, SameRequesterCanRenewAndRelease) {
  follower::ControlGuardConfig config;
  config.require_lease = true;
  config.maximum_lease_sec = 10.0;
  follower::ControlGuard guard(config);
  EXPECT_TRUE(guard.acquire("mission", "command_only", 1.0, 5.0).success);
  EXPECT_TRUE(guard.acquire("mission", "command_only", 2.0, 5.5).success);
  EXPECT_NEAR(2.0, guard.remaining(5.5), 1e-9);
  EXPECT_FALSE(guard.release("ui", 5.5).success);
  EXPECT_TRUE(guard.release("mission", 5.5).success);
  EXPECT_FALSE(guard.authorized(5.5));
}

TEST(ControlGuard, RejectsInvalidLeaseArguments) {
  follower::ControlGuardConfig config;
  config.require_lease = true;
  config.minimum_lease_sec = 0.2;
  config.maximum_lease_sec = 2.0;
  follower::ControlGuard guard(config);
  EXPECT_FALSE(guard.acquire("", "command_only", 1.0, 0.0).success);
  EXPECT_FALSE(guard.acquire("ui", "command_only", 0.1, 0.0).success);
  EXPECT_FALSE(guard.acquire("ui", "command_only", 3.0, 0.0).success);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
