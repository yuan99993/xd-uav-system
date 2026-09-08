#include <gtest/gtest.h>

#include <mavros_msgs/ExtendedState.h>

#include <xd_uav_control_manager/vtol_vehicle_adapter.h>
#include <xd_uav_control_manager/fixedwing_vehicle_adapter.h>
#include <xd_uav_control_manager/multirotor_vehicle_adapter.h>
#include <xd_uav_control_manager/vtol_landing_coordinator.h>
#include <xd_uav_control_manager/vtol_takeoff_coordinator.h>

namespace manager = xd_uav_control_manager;
namespace control = xd_uav_controller;

TEST(VtolVehicleAdapter, MapsObservedPx4StateWithoutInventingTransitions) {
  manager::VtolVehicleAdapter adapter(control::AirframeType::kVtol);

  EXPECT_EQ(adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_MC, true),
            control::FlightRegime::kHover);
  EXPECT_EQ(adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_TRANSITION_TO_FW,
                            true),
            control::FlightRegime::kTransitionToForward);
  EXPECT_EQ(adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_FW, true),
            control::FlightRegime::kForwardFlight);
  EXPECT_EQ(adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_UNDEFINED,
                            false),
            control::FlightRegime::kUnknown);
}

TEST(VtolVehicleAdapter, ServiceAcceptanceKeepsObservedRegimeUntilPx4Confirms) {
  manager::VtolVehicleAdapter adapter(control::AirframeType::kVtol);
  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_MC, true);

  std::string reason;
  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(10.0), &reason));
  EXPECT_TRUE(adapter.transitionStatus().pending);
  EXPECT_EQ(adapter.observedRegime(), control::FlightRegime::kHover);

  adapter.recordServiceAccepted(0U);
  EXPECT_EQ(adapter.observedRegime(), control::FlightRegime::kHover);

  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_FW, true);
  EXPECT_FALSE(adapter.transitionStatus().pending);
  EXPECT_EQ(adapter.observedRegime(), control::FlightRegime::kForwardFlight);
}

TEST(VtolVehicleAdapter, TimesOutUnconfirmedTransition) {
  manager::VtolVehicleAdapter adapter(control::AirframeType::kVtol);
  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_MC, true);
  std::string reason;
  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(10.0), &reason));

  EXPECT_FALSE(adapter.checkTimeout(ros::Time(19.9), 10.0));
  EXPECT_TRUE(adapter.checkTimeout(ros::Time(20.1), 10.0));
  EXPECT_FALSE(adapter.transitionStatus().pending);
  EXPECT_TRUE(adapter.transitionStatus().timed_out);
}

TEST(VtolVehicleAdapter, PendingRequestsAreIdempotentButCannotReverse) {
  manager::VtolVehicleAdapter adapter(control::AirframeType::kVtol);
  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_MC, true);
  std::string reason;
  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(10.0), &reason));
  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(15.0), &reason));
  EXPECT_EQ(adapter.transitionStatus().started_at, ros::Time(10.0));
  EXPECT_FALSE(adapter.beginTransition(control::RequestedRegime::kHover,
                                       ros::Time(15.0), &reason));
}

TEST(VtolVehicleAdapter, PreservesTransportAckAndCompletionResults) {
  manager::VtolVehicleAdapter adapter(control::AirframeType::kVtol);
  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_MC, true);
  std::string reason;
  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(10.0), &reason));
  adapter.recordServiceFailure(37U, "PX4 rejected");
  EXPECT_EQ(adapter.transitionStatus().last_result,
            manager::TransitionResult::kAckRejected);
  EXPECT_EQ(adapter.transitionStatus().raw_ack_result, 37U);
  EXPECT_EQ(adapter.observedRegime(), control::FlightRegime::kHover);

  ASSERT_TRUE(adapter.beginTransition(control::RequestedRegime::kForwardFlight,
                                      ros::Time(20.0), &reason));
  adapter.recordServiceAccepted(5U);
  EXPECT_EQ(adapter.transitionStatus().last_result,
            manager::TransitionResult::kAccepted);
  adapter.observe(mavros_msgs::ExtendedState::VTOL_STATE_FW, true);
  EXPECT_EQ(adapter.transitionStatus().last_result,
            manager::TransitionResult::kCompleted);
  EXPECT_EQ(adapter.transitionStatus().target,
            control::RequestedRegime::kNone);
  EXPECT_EQ(adapter.transitionStatus().last_target,
            control::RequestedRegime::kForwardFlight);
}

TEST(VtolLandingCoordinator, ForwardFlightMustConfirmHoverBeforeDescent) {
  manager::VtolLandingCoordinator landing;
  std::string reason;
  ASSERT_TRUE(landing.start(control::FlightRegime::kForwardFlight, &reason));
  EXPECT_EQ(landing.phase(), manager::VtolLandingPhase::kRequestHover);
  landing.transitionRequested();
  EXPECT_EQ(landing.phase(), manager::VtolLandingPhase::kWaitHover);
  EXPECT_FALSE(landing.observe(control::FlightRegime::kTransitionToHover));
  EXPECT_TRUE(landing.observe(control::FlightRegime::kHover));
  EXPECT_EQ(landing.phase(), manager::VtolLandingPhase::kVerticalDescent);
}

TEST(VtolLandingCoordinator, HoverStartsDirectlyAndFailureCannotDescend) {
  manager::VtolLandingCoordinator landing;
  std::string reason;
  ASSERT_TRUE(landing.start(control::FlightRegime::kHover, &reason));
  EXPECT_EQ(landing.phase(), manager::VtolLandingPhase::kVerticalDescent);

  landing.reset();
  ASSERT_TRUE(landing.start(control::FlightRegime::kForwardFlight, &reason));
  landing.transitionRequested();
  landing.fail("transition rejected");
  EXPECT_EQ(landing.phase(), manager::VtolLandingPhase::kFailed);
  EXPECT_FALSE(landing.observe(control::FlightRegime::kHover));
}

TEST(VtolTakeoffCoordinator, ForwardFlightMustConfirmHoverBeforeTakeoff) {
  manager::VtolTakeoffCoordinator takeoff;
  std::string reason;
  ASSERT_TRUE(takeoff.start(control::FlightRegime::kForwardFlight, &reason));
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kRequestHover);
  takeoff.transitionRequested();
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kWaitHover);
  EXPECT_FALSE(takeoff.observe(control::FlightRegime::kTransitionToHover));
  EXPECT_TRUE(takeoff.observe(control::FlightRegime::kHover));
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kReady);
  takeoff.commandStarted();
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kStarted);
}

TEST(VtolTakeoffCoordinator, HoverStartsReadyAndUnknownIsRejected) {
  manager::VtolTakeoffCoordinator takeoff;
  std::string reason;
  ASSERT_TRUE(takeoff.start(control::FlightRegime::kHover, &reason));
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kReady);

  takeoff.reset();
  EXPECT_FALSE(takeoff.start(control::FlightRegime::kUnknown, &reason));
  EXPECT_EQ(takeoff.phase(), manager::VtolTakeoffPhase::kFailed);
}

TEST(VehicleAdapterCapabilities, DeclareBehaviorWithoutAirframeStringBranches) {
  manager::MultirotorVehicleAdapter multirotor;
  manager::FixedWingVehicleAdapter fixed_wing;
  manager::VtolVehicleAdapter vtol(control::AirframeType::kVtol);

  EXPECT_FALSE(multirotor.capabilities().supports_transition);
  EXPECT_TRUE(multirotor.capabilities().supports_vertical_takeoff);
  EXPECT_FALSE(multirotor.requiresAirspeed(control::FlightRegime::kHover));

  EXPECT_FALSE(fixed_wing.capabilities().supports_transition);
  EXPECT_FALSE(fixed_wing.capabilities().supports_vertical_landing);
  EXPECT_TRUE(
      fixed_wing.requiresAirspeed(control::FlightRegime::kForwardFlight));

  EXPECT_TRUE(vtol.capabilities().supports_transition);
  EXPECT_TRUE(vtol.capabilities().supports_vertical_landing);
  EXPECT_FALSE(vtol.requiresAirspeed(control::FlightRegime::kHover));
  EXPECT_TRUE(
      vtol.requiresAirspeed(control::FlightRegime::kForwardFlight));
  EXPECT_TRUE(
      vtol.requiresAirspeed(control::FlightRegime::kTransitionToForward));
  EXPECT_TRUE(
      vtol.requiresAirspeed(control::FlightRegime::kTransitionToHover));
}
