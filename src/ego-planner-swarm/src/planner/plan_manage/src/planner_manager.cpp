// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() {}

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);
    nh.param("manager/astar_resolution", a_star_resolution_, 0.2);
    double a_star_search_time = 0.15;
    int a_star_pool_size_xy = 100;
    int a_star_pool_size_z = 60;
    nh.param("manager/astar_search_time", a_star_search_time, 0.15);
    nh.param("manager/astar_pool_size_xy", a_star_pool_size_xy, 100);
    nh.param("manager/astar_pool_size_z", a_star_pool_size_z, 60);
    a_star_pool_size_xy = std::max(20, a_star_pool_size_xy);
    a_star_pool_size_z = std::max(20, a_star_pool_size_z);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);
    a_star_resolution_ = std::max(grid_map_->getResolution(),
                                  a_star_resolution_);

    // obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
    // obj_predictor_->init();
    // obj_pub_ = nh.advertise<visualization_msgs::Marker>("/dynamic/obj_prdi", 10); // zx-todo

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);
    bspline_optimizer_->setAStarResolution(a_star_resolution_);
    bspline_optimizer_->a_star_.reset(new AStar);
    bspline_optimizer_->a_star_->initGridMap(
        grid_map_, Eigen::Vector3i(a_star_pool_size_xy,
                                  a_star_pool_size_xy,
                                  a_star_pool_size_z));
    bspline_optimizer_->a_star_->setSearchTimeLimit(a_star_search_time);
    ROS_INFO("EGO A*: resolution %.2f m, search box %.1f x %.1f x %.1f m, timeout %.2f s",
             a_star_resolution_,
             a_star_resolution_ * a_star_pool_size_xy,
             a_star_resolution_ * a_star_pool_size_xy,
             a_star_resolution_ * a_star_pool_size_z,
             std::max(0.01, a_star_search_time));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    const std::vector<Eigen::Vector3d> no_route_reference;
    return reboundReplan(start_pt, start_vel, start_acc, local_target_pt,
                         local_target_vel, flag_polyInit, flag_randomPolyTraj,
                         no_route_reference);
  }

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> &route_reference)
  {
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n", pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    const bool use_route_reference = route_reference.size() >= 2;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (use_route_reference)
      {
        // In route mode the local optimizer must start from the same geometry
        // that the allocator supplied.  The legacy polynomial-only seed goes
        // straight to local_target_pt and silently cuts every route corner.
        for (const Eigen::Vector3d &reference_point : route_reference)
        {
          if (!reference_point.allFinite())
            continue;
          if (point_set.empty() ||
              (reference_point - point_set.back()).norm() >= 0.05)
            point_set.push_back(reference_point);
        }
        if (point_set.empty())
          point_set.push_back(start_pt);
        point_set.front() = start_pt;
        if ((point_set.back() - local_target_pt).norm() >= 0.05)
          point_set.push_back(local_target_pt);
        else
          point_set.back() = local_target_pt;

        // The allocator path is a geometric reference, not a collision-free
        // local trajectory. If its first horizon is blocked, starting the
        // optimizer from that same colliding polyline can leave the first
        // B-spline interval inside the inflated map. In that situation use a
        // short A* detour as the local seed. The global allocator route is
        // unchanged and will be used again after the vehicle clears the wall.
        const double collision_sample_step =
            std::max(0.05, grid_map_->getResolution() * 0.5);
        bool route_seed_blocked = false;
        for (size_t i = 1; i < point_set.size() && !route_seed_blocked; ++i)
        {
          const Eigen::Vector3d delta = point_set[i] - point_set[i - 1];
          const double length = delta.norm();
          const int samples = std::max(
              1, static_cast<int>(std::ceil(length / collision_sample_step)));
          for (int sample = 0; sample <= samples; ++sample)
          {
            const double ratio = static_cast<double>(sample) / samples;
            const Eigen::Vector3d position =
                point_set[i - 1] + ratio * delta;
            if (grid_map_->getInflateOccupancy(position) != 0)
            {
              route_seed_blocked = true;
              break;
            }
          }
        }

        if (route_seed_blocked && bspline_optimizer_->a_star_ &&
            bspline_optimizer_->a_star_->AstarSearch(
                a_star_resolution_, start_pt, local_target_pt))
        {
          const std::vector<Eigen::Vector3d> astar_path =
              bspline_optimizer_->a_star_->getPath();
          std::vector<Eigen::Vector3d> detour;
          detour.reserve(astar_path.size());

          // Keep approximately the normal control-point spacing, while
          // retaining sharp A* corners so the resampling does not cut back
          // through the obstacle.
          const double detour_spacing =
              std::max(0.25, pp_.ctrl_pt_dist * 0.8);
          for (size_t i = 0; i < astar_path.size(); ++i)
          {
            if (!astar_path[i].allFinite())
              continue;
            bool keep = detour.empty() ||
                        (astar_path[i] - detour.back()).norm() >=
                            detour_spacing;
            if (!keep && i > 0 && i + 1 < astar_path.size())
            {
              const Eigen::Vector3d incoming =
                  astar_path[i] - astar_path[i - 1];
              const Eigen::Vector3d outgoing =
                  astar_path[i + 1] - astar_path[i];
              if (incoming.squaredNorm() > 1.0e-8 &&
                  outgoing.squaredNorm() > 1.0e-8 &&
                  incoming.normalized().dot(outgoing.normalized()) < 0.85)
                keep = true;
            }
            if (keep)
              detour.push_back(astar_path[i]);
          }

          const bool reaches_target =
              detour.size() >= 2 &&
              (detour.back() - local_target_pt).norm() <= 0.35;
          if (reaches_target)
          {
            detour.front() = start_pt;
            detour.back() = local_target_pt;
            if (detour.size() >= 7)
            {
              point_set.swap(detour);
              ROS_WARN_THROTTLE(
                  1.0,
                  "Allocator route is blocked; using a local A* detour seed");
            }
          }
        }

        // parameterizeToBspline() is numerically better conditioned with a
        // small number of samples even for a very short final horizon. Keep
        // the route geometry while doing so: interpolating only start/end
        // points would turn a short L-shaped route into a chord outside the
        // allocator's Path.
        if (point_set.size() < 7)
        {
          const vector<Eigen::Vector3d> route_seed = point_set;
          vector<double> arc_length(route_seed.size(), 0.0);
          for (size_t i = 1; i < route_seed.size(); ++i)
          {
            arc_length[i] = arc_length[i - 1] +
                            (route_seed[i] - route_seed[i - 1]).norm();
          }

          const double route_length = arc_length.back();
          point_set.clear();
          for (int sample = 0; sample < 7; ++sample)
          {
            const double sample_length =
                route_length * static_cast<double>(sample) / 6.0;
            size_t segment = 0;
            while (segment + 1 < arc_length.size() &&
                   arc_length[segment + 1] < sample_length)
              ++segment;

            const double segment_length =
                arc_length[segment + 1] - arc_length[segment];
            const double ratio = segment_length > 1.0e-6
                                     ? (sample_length - arc_length[segment]) /
                                           segment_length
                                     : 0.0;
            point_set.push_back(route_seed[segment] * (1.0 - ratio) +
                                route_seed[segment + 1] * ratio);
          }
          point_set.front() = start_pt;
          point_set.back() = local_target_pt;
        }
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());
      }
      else if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    if (use_route_reference)
      bspline_optimizer_->setRouteReference(point_set);
    else
      bspline_optimizer_->clearRouteReference();

    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;
    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;

    if (pp_.use_distinctive_trajs)
    {
      // cout << "enter" << endl;
      std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          flag_step_1_success = true;
          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            ctrl_pts = ctrl_pts_temp;
          }

          // visualization
          point_set.clear();
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
      t_opt = ros::Time::now() - t_start;
      //static int vis_id = 0;
      visualization_->displayInitPathList(point_set, 0.2, 0);
    }

    cout << "plan_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      visualization_->displayOptimalList(ctrl_pts, 0);
      continous_failures_count_++;
      return false;
    }

    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // Note: Only adjust time in single drone mode. But we still allow drone_0 to adjust its time profile.
    if (pp_.drone_id <= 0)
    {

      double ratio;
      bool flag_step_2_success = true;
      if (!pos.checkFeasibility(ratio, false))
      {
        cout << "Need to reallocate time." << endl;

        Eigen::MatrixXd optimal_control_points;
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
        if (flag_step_2_success)
          pos = UniformBspline(optimal_control_points, 3, ts);
      }

      if (!flag_step_2_success)
      {
        printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
        continous_failures_count_++;
        return false;
      }
    }
    else
    {
      static bool print_once = true;
      if (print_once)
      {
        print_once = false;
        ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
      }
    }

    t_refine = ros::Time::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << ",avg_time=" << sum_time / count_success << endl;

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (local_data_.start_time_.toSec() < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = local_data_.start_time_.toSec();
    double other_traj_start_time = swarm_trajs_buf_[drone_id].start_time_.toSec();

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + local_data_.duration_ * 2 / 3, other_traj_start_time + swarm_trajs_buf_[drone_id].duration_);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((local_data_.position_traj_.evaluateDeBoorT(t - my_traj_start_time) - swarm_trajs_buf_[drone_id].position_traj_.evaluateDeBoorT(t - other_traj_start_time)).norm() < bspline_optimizer_->getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    if (waypoints.empty() || !start_pos.allFinite() ||
        !start_vel.allFinite() || !start_acc.allFinite() ||
        !end_vel.allFinite() || !end_acc.allFinite() ||
        pp_.max_vel_ <= 1.0e-3)
    {
      ROS_ERROR("Cannot generate global trajectory: invalid route or max velocity");
      return false;
    }

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    // The route handoff normally includes the measured position as its first
    // pose. Remove only near duplicates; every meaningful route bend remains
    // a constraint of the global trajectory.
    constexpr double MIN_WAYPOINT_DISTANCE = 0.05;
    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      if (!waypoints[wp_i].allFinite())
      {
        ROS_ERROR("Cannot generate global trajectory: non-finite route point");
        return false;
      }
      if ((waypoints[wp_i] - points.back()).norm() >= MIN_WAYPOINT_DISTANCE)
        points.push_back(waypoints[wp_i]);
    }

    if (points.size() < 2)
      return false;

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
      time(i) = std::max(time(i), 0.05);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // A minimum-snap polynomial through a lawn-mower route overshoots every
    // sharp reversal: its continuous derivatives make the global reference
    // leave the allocator's Path before the local obstacle planner even runs.
    // Keep the route-mode global reference on the supplied piecewise-linear
    // geometry.  The local B-spline still smooths the active horizon and can
    // leave this reference only when the occupancy map requires an avoidance
    // manoeuvre.
    PolynomialTraj gl_traj;
    for (int i = 0; i < pt_num - 1; ++i)
    {
      const Eigen::Vector3d delta = pos.col(i + 1) - pos.col(i);
      const double segment_time = time(i);
      const Eigen::Vector3d velocity = delta / segment_time;

      // PolynomialTraj stores coefficients from highest to lowest power for
      // addSegment(): [slope, intercept] gives p(t) = intercept + slope * t.
      // A linear segment is monotonic and therefore cannot cut outside its
      // two route endpoints.
      gl_traj.addSegment(
          {velocity.x(), pos(0, i)},
          {velocity.y(), pos(1, i)},
          {velocity.z(), pos(2, i)},
          segment_time);
    }
    gl_traj.init();

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    const int reference_segments = std::max(1, static_cast<int>(ctrl_pts.cols()) - 3);
    double t_step = traj.getTimeSum() / reference_segments;
    bspline_optimizer_->ref_pts_.clear();
    for (int i = 0; i <= reference_segments; ++i)
    {
      const double t = std::min(traj.getTimeSum(), i * t_step);
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));
    }

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
