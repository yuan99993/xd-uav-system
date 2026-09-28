
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    have_recv_pre_agent_ = false;
    have_pending_reference_path_ = false;
    flag_escape_emergency_ = false;
    route_reference_points_.clear();
    route_progress_time_ = 0.0;
    route_target_time_ = 0.0;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/replan_lookahead_time", replan_lookahead_time_, 0.12);
    replan_lookahead_time_ = std::max(0.0, replan_lookahead_time_);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/safety_check_interval", safety_check_interval_, 0.05);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/reference_path_topic", reference_path_topic_,
             std::string("reference_path"));
    nh.param("fsm/reference_path_frame", reference_path_frame_,
             std::string("world"));

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->setDroneIdtoOpt();

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(
        ros::Duration(std::max(0.005, safety_check_interval_)),
        &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

    if (planner_manager_->pp_.drone_id >= 1)
    {
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::REFENCE_PATH)
    {
      reference_path_sub_ = nh.subscribe(
          reference_path_topic_, 1, &EGOReplanFSM::referencePathCallback,
          this, ros::TransportHints().tcpNoDelay());
      // A complete Path is the trigger in route mode.  The actual planning
      // callback still waits for odometry, so a latched task Path is safe to
      // publish before this node has finished starting.
      have_trigger_ = true;
      ROS_INFO("EGO route mode enabled; waiting for Path on [%s]",
               reference_path_topic_.c_str());
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    // Planning already validates frame and finite coordinates. Preserve the
    // complete 3-D task goal instead of forcing every live target to z=1 m.
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z);

    planNextWaypoint(end_wp);
  }

  void EGOReplanFSM::referencePathCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (target_type_ != TARGET_TYPE::REFENCE_PATH || !msg)
      return;

    // The planning bridge publishes an empty latched Path on cancellation.
    // Clear EGO's target as well, otherwise traj_server would keep publishing
    // the last B-spline after the allocator released the task.
    if (msg->poses.empty())
    {
      clearReferencePath();
      return;
    }

    if (!have_odom_)
    {
      pending_reference_path_ = *msg;
      have_pending_reference_path_ = true;
      ROS_INFO_THROTTLE(2.0,
                        "EGO route received before odometry; buffering it");
      return;
    }

    setReferencePath(*msg);
  }

  bool EGOReplanFSM::setReferencePath(const nav_msgs::Path &msg)
  {
    auto canonical_frame = [](const std::string &frame) {
      const std::string::size_type first = frame.find_first_not_of('/');
      if (first == std::string::npos)
        return std::string();
      const std::string::size_type last = frame.find_last_not_of('/');
      return frame.substr(first, last - first + 1);
    };

    if (canonical_frame(msg.header.frame_id) !=
        canonical_frame(reference_path_frame_))
    {
      ROS_ERROR("Rejecting EGO reference Path: frame [%s] != [%s]",
                msg.header.frame_id.c_str(), reference_path_frame_.c_str());
      return false;
    }
    if (msg.header.stamp.toSec() <= 0.0)
    {
      ROS_ERROR("Rejecting EGO reference Path with non-positive timestamp");
      return false;
    }
    const double path_age = (ros::Time::now() - msg.header.stamp).toSec();
    if (path_age < -0.25 || path_age > 2.5)
    {
      ROS_WARN("Rejecting stale/future EGO reference Path (age=%.3fs)",
               path_age);
      return false;
    }

    std::vector<Eigen::Vector3d> route;
    route.reserve(msg.poses.size());
    constexpr double MIN_ROUTE_POINT_DISTANCE = 0.05;
    for (const auto &pose : msg.poses)
    {
      if (!pose.header.frame_id.empty() &&
          canonical_frame(pose.header.frame_id) !=
              canonical_frame(reference_path_frame_))
      {
        ROS_ERROR("Rejecting EGO reference Path: nested pose frame [%s] != [%s]",
                  pose.header.frame_id.c_str(), reference_path_frame_.c_str());
        return false;
      }
      const Eigen::Vector3d point(pose.pose.position.x,
                                  pose.pose.position.y,
                                  pose.pose.position.z);
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          !std::isfinite(point.z()))
      {
        ROS_ERROR("Rejecting EGO reference Path containing non-finite point");
        return false;
      }
      if (route.empty() ||
          (point - route.back()).norm() >= MIN_ROUTE_POINT_DISTANCE)
      {
        route.push_back(point);
      }
    }

    if (route.size() < 2)
    {
      ROS_ERROR("Rejecting EGO reference Path: fewer than two distinct points");
      return false;
    }

    // Generate one continuous global trajectory through all route points.
    // EGO's existing rebound optimizer then replans only the local horizon
    // against the rolling occupancy map built from the native point cloud.
    if (!planner_manager_->planGlobalTrajWaypoints(
            odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), route,
            Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()))
    {
      ROS_ERROR("EGO failed to create a global trajectory from %zu Path points",
                route.size());
      return false;
    }

    wps_ = route;
    route_reference_points_.clear();
    route_progress_time_ = 0.0;
    route_target_time_ = 0.0;
    wp_id_ = 0;
    end_pt_ = route.back();
    end_vel_.setZero();
    have_target_ = true;
    have_trigger_ = true;
    have_new_target_ = true;
    flag_escape_emergency_ = false;

    // Reconnect a newly received/recovered route from measured odometry
    // immediately instead of waiting for the previous local trajectory.
    if (exec_state_ != INIT)
      changeFSMExecState(GEN_NEW_TRAJ, "REFERENCE_PATH");

    const double display_step = 0.2;
    const int display_count = std::min(
        2000, static_cast<int>(std::ceil(
                  planner_manager_->global_data_.global_duration_ /
                  display_step)));
    std::vector<Eigen::Vector3d> global_path;
    global_path.reserve(std::max(0, display_count));
    for (int i = 0; i < display_count; ++i)
    {
      global_path.push_back(planner_manager_->global_data_.global_traj_.evaluate(
          std::min(planner_manager_->global_data_.global_duration_,
                   i * display_step)));
    }
    if (!global_path.empty())
      visualization_->displayGlobalPathList(global_path, 0.1, 0);

    double route_length = 0.0;
    for (size_t i = 1; i < route.size(); ++i)
      route_length += (route[i] - route[i - 1]).norm();
    ROS_INFO("EGO accepted complete reference Path: %zu points, %.2f m, %.2f s",
             route.size(), route_length,
             planner_manager_->global_data_.global_duration_);
    return true;
  }

  void EGOReplanFSM::clearReferencePath()
  {
    pending_reference_path_.poses.clear();
    have_pending_reference_path_ = false;
    if (target_type_ != TARGET_TYPE::REFENCE_PATH)
      return;

    have_target_ = false;
    have_new_target_ = false;
    wps_.clear();
    route_reference_points_.clear();
    route_progress_time_ = 0.0;
    route_target_time_ = 0.0;
    if (have_odom_ && planner_manager_)
    {
      // Cancellation is a deliberate handoff to the baseline controller, not
      // a safety failure. Publish a stationary trajectory before waiting for
      // the next route so a standalone EGO instance also stops cleanly.
      callEmergencyStop(odom_pos_);
    }
    if (exec_state_ != INIT && exec_state_ != WAIT_TARGET)
      changeFSMExecState(WAIT_TARGET, "REFERENCE_PATH_CLEAR");
    ROS_INFO("EGO reference Path cleared; waiting for the next task route");
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;

    if (target_type_ == TARGET_TYPE::REFENCE_PATH &&
        have_pending_reference_path_)
    {
      nav_msgs::Path pending = pending_reference_path_;
      have_pending_reference_path_ = false;
      pending_reference_path_.poses.clear();
      setReferencePath(pending);
    }
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          // A complete allocator route is retried by the 10 ms FSM timer.
          // Do not perform ten full A*/optimizer attempts in one callback:
          // that blocks cloud and safety callbacks and makes the aircraft
          // appear to stop responding. Keep the legacy retry count for the
          // old manual/preset modes.
          const int initial_plan_trials =
              target_type_ == TARGET_TYPE::REFENCE_PATH ? 1 : 10;
          bool success = planFromGlobalTraj(initial_plan_trials);
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      const int initial_plan_trials =
          target_type_ == TARGET_TYPE::REFENCE_PATH ? 1 : 10;
      bool success = planFromGlobalTraj(initial_plan_trials);
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          have_target_ = false;
          have_trigger_ = false;

          if (target_type_ == TARGET_TYPE::PRESET_TARGET)
          {
            wp_id_ = 0;
            planNextWaypoint(wps_[wp_id_]);
          }

          changeFSMExecState(WAIT_TARGET, "FSM");
          goto force_return;
          // return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    // Plan the splice from a short distance into the trajectory that is still
    // being executed. This compensates for A*/L-BFGS computation and message
    // handoff latency; starting from the state at callback entry otherwise
    // makes every replacement trajectory begin behind the vehicle.
    const double t_plan = std::max(
        0.0, std::min(info->duration_,
                      t_cur + replan_lookahead_time_));

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_plan);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_plan);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_plan);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // EmergencyStop() replaces the active trajectory with a stationary
    // trajectory at the measured vehicle position. Do not feed that safety
    // trajectory back into the normal collision checker: a lidar return on
    // the vehicle (or the occupied voxel under it) would immediately issue
    // another EMERGENCY_STOP and prevent the FSM from ever attempting its
    // recovery replan.
    if (exec_state_ == WAIT_TARGET || exec_state_ == EMERGENCY_STOP ||
        info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      if (exec_state_ != EMERGENCY_STOP)
      {
        ROS_ERROR_THROTTLE(1.0, "Depth/cloud input lost! EMERGENCY_STOP");
        // EMERGENCY_STOP must publish one stationary trajectory before the
        // recovery branch is allowed to generate a new route.
        flag_escape_emergency_ = true;
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      }
      return;
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    // Check the complete active local trajectory. The old first-2/3 rule was
    // tuned for a short-horizon free-goal demo, but in route mode it allowed
    // the last third of the allocator path to remain blocked and still be
    // sent to the controller.
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      bool occ = false;
      const Eigen::Vector3d predicted_position =
          info->position_traj_.evaluateDeBoorT(t);
      // Unknown voxels inside the rolling map are traversable, but -1 means
      // the trajectory has left the finite collision-checking volume and must
      // trigger replanning just like an occupied voxel.
      occ |= map->getInflateOccupancy(predicted_position) != 0;

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        // Both trajectories must be evaluated at the same future time. Using
        // p_cur here reports false collisions whenever another aircraft's
        // future position passes near our current position.
        double dist = (predicted_position - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            // The emergency state is entered from an active trajectory, so
            // request the stationary safety trajectory exactly once.
            flag_escape_emergency_ = true;
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    const std::vector<Eigen::Vector3d> empty_route_reference;
    const std::vector<Eigen::Vector3d> &route_reference =
        target_type_ == TARGET_TYPE::REFENCE_PATH
            ? route_reference_points_
            : empty_route_reference;
    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_,
                                        local_target_pt_, local_target_vel_,
                                        (have_new_target_ || flag_use_poly_init),
                                        flag_randomPolyTraj, route_reference);
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }

  void EGOReplanFSM::getLocalTarget()
  {
    if (target_type_ == TARGET_TYPE::REFENCE_PATH &&
        planner_manager_->global_data_.global_duration_ > 1.0e-3)
    {
      // Route mode advances on the geometric length of the reference rather
      // than the Euclidean distance to a future point.  Euclidean look-ahead
      // becomes ambiguous at lawn-mower turns and can select a point on the
      // wrong side of a bend.
      const double duration = planner_manager_->global_data_.global_duration_;
      const double max_vel = std::max(0.1, planner_manager_->pp_.max_vel_);
      const double previous_progress = std::max(
          0.0, std::min(duration, route_progress_time_));
      const double previous_target = std::max(
          previous_progress,
          std::min(duration,
                   planner_manager_->global_data_.last_progress_time_));
      const double search_back = std::max(1.0, planning_horizen_ / max_vel * 0.25);
      const double search_forward = std::max(2.0, planning_horizen_ / max_vel);
      const double search_start = std::max(0.0, previous_progress - search_back);
      const double search_end = std::min(
          duration, std::max(previous_progress + search_forward,
                             previous_target));
      const double search_step = 0.05;

      double closest_time = previous_progress;
      double closest_distance =
          (planner_manager_->global_data_.global_traj_.evaluate(
               previous_progress) - start_pt_)
              .norm();
      for (double candidate = search_start; candidate <= search_end + 1.0e-6;
           candidate += search_step)
      {
        const double candidate_time = std::min(duration, candidate);
        const double distance =
            (planner_manager_->global_data_.global_traj_.evaluate(candidate_time) -
             start_pt_)
                .norm();
        if (distance < closest_distance)
        {
          closest_distance = distance;
          closest_time = candidate_time;
        }
      }

      // Do not let a temporary tracking error move the route cursor backward.
      route_progress_time_ = std::max(previous_progress, closest_time);

      const double local_horizon = std::max(0.5, planning_horizen_);
      double target_time = duration;
      double route_distance = 0.0;
      Eigen::Vector3d previous_position =
          planner_manager_->global_data_.global_traj_.evaluate(
              route_progress_time_);
      for (double candidate = route_progress_time_ + search_step;
           candidate < duration + 1.0e-6; candidate += search_step)
      {
        const double candidate_time = std::min(duration, candidate);
        const Eigen::Vector3d candidate_position =
            planner_manager_->global_data_.global_traj_.evaluate(candidate_time);
        route_distance += (candidate_position - previous_position).norm();
        previous_position = candidate_position;
        if (route_distance >= local_horizon)
        {
          target_time = candidate_time;
          break;
        }
      }

      route_target_time_ = target_time;
      local_target_pt_ = planner_manager_->global_data_.global_traj_.evaluate(
          target_time);
      planner_manager_->global_data_.last_progress_time_ = target_time;

      const double braking_distance =
          planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_ /
          (2.0 * std::max(0.1, planner_manager_->pp_.max_acc_));
      if (target_time >= duration - 1.0e-5 ||
          (end_pt_ - local_target_pt_).norm() < braking_distance)
      {
        local_target_vel_.setZero();
      }
      else
      {
        local_target_vel_ =
            planner_manager_->global_data_.global_traj_.evaluateVel(target_time);
      }

      // The local B-spline is initialized from the same continuous reference
      // that EGO uses for look-ahead. This prevents the first optimization
      // iteration from replacing a curved/turning route with a straight chord.
      route_reference_points_.clear();
      const double reference_spacing = 0.35;
      const int reference_count = std::max(
          6, static_cast<int>(std::ceil(
                  std::max(route_distance, local_horizon) /
                  reference_spacing)));
      route_reference_points_.reserve(reference_count + 1);
      route_reference_points_.push_back(start_pt_);
      for (int i = 1; i <= reference_count; ++i)
      {
        const double ratio = static_cast<double>(i) / reference_count;
        const double sample_time =
            route_progress_time_ + (target_time - route_progress_time_) * ratio;
        const Eigen::Vector3d sample =
            planner_manager_->global_data_.global_traj_.evaluate(
                std::min(duration, sample_time));
        if ((sample - route_reference_points_.back()).norm() >= 0.05 ||
            i == reference_count)
        {
          route_reference_points_.push_back(sample);
        }
      }
      route_reference_points_.back() = local_target_pt_;
      return;
    }

    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
