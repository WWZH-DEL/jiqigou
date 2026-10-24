/*
 * Adapted for pure path tracking with local A* obstacle avoidance.
 * Original: Copyright 2015-2019 Autoware Foundation.
 *
 * Architecture:
 *   - Subscribes /global_path (nav_msgs::Path) — recorded trajectory
 *   - Subscribes /scan (sensor_msgs::LaserScan) — LiDAR for obstacle detection
 *   - Builds 5m×4m local costmap following the car in map frame
 *   - Detects if obstacles block the upcoming path
 *   - Runs Hybrid A* to plan around blocked segments
 *   - Publishes /safety_waypoints (nav_msgs::Path) for the controller
 *   - On A* failure, publishes global_path (test.py handles obstacle braking)
 */

#include "astar_search/astar_avoid.h"

#include <limits>
#include <algorithm>
#include <utility>

// ============================================================================
// Constructor / Destructor
// ============================================================================

AstarAvoid::AstarAvoid()
  : nh_()
  , private_nh_("~")
  , terminate_thread_(false)
  , closest_local_index_(-1)
  , end_of_avoid_index_(-1)
  , last_goal_idx_(-1)
  , global_path_initialized_(false)
  , scan_initialized_(false)
  , test_obstacle_detected_(false)
  , is_loop_path_(false)
{
  // ---- local costmap params ----
  private_nh_.param<double>("local_costmap_width", local_costmap_width_, 8.0);
  private_nh_.param<double>("local_costmap_height", local_costmap_height_, 8.0);
  private_nh_.param<double>("costmap_resolution", costmap_resolution_, 0.05);
  private_nh_.param<std::string>("laser_frame", laser_frame_, "body");
  private_nh_.param<std::string>("base_frame", base_frame_, "body_foot");

  // ---- obstacle detection params ----
  private_nh_.param<double>("obstacle_check_distance", obstacle_check_distance_, 1.5);
  private_nh_.param<double>("obstacle_on_path_threshold", obstacle_on_path_threshold_, 0.2);
  private_nh_.param<double>("obstacle_inflation_radius", obstacle_inflation_radius_, 0.3);
  private_nh_.param<double>("loop_closure_threshold", loop_closure_threshold_, 0.5);

  // ---- avoidance params ----
  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, true);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 1.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 0.5);
  private_nh_.param<double>("a_star_goal_lookahead", a_star_goal_lookahead_, 0.8);
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);

  // ---- publishers ----
  safety_waypoints_pub_ = nh_.advertise<nav_msgs::Path>("safety_waypoints", 1, true);
  local_costmap_pub_   = nh_.advertise<nav_msgs::OccupancyGrid>("local_costmap", 1, true);
  astar_path_pub_      = nh_.advertise<nav_msgs::Path>("astar_path", 1, true);
  avoidance_state_pub_ = nh_.advertise<std_msgs::Int32>("avoidance_state", 1, true);

  // ---- subscribers ----
  global_path_sub_      = nh_.subscribe("global_path", 1, &AstarAvoid::globalPathCallback, this);
  scan_sub_             = nh_.subscribe("scan", 1, &AstarAvoid::scanCallback, this);
  obstacle_signal_sub_  = nh_.subscribe("obstacle_stop", 1, &AstarAvoid::obstacleSignalCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

AstarAvoid::~AstarAvoid()
{
  if (publish_thread_.joinable())
    publish_thread_.join();
}

// ============================================================================
// Callbacks
// ============================================================================

void AstarAvoid::globalPathCallback(const nav_msgs::Path& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);  // publishWaypoints() 线程会读 global_path_

  // Detect actual path change by comparing first+last pose positions
  bool path_changed = false;
  if (!global_path_initialized_)
  {
    path_changed = true;
  }
  else if (msg.poses.size() != global_path_.poses.size())
  {
    path_changed = true;  // different pose count
  }
  else if (!msg.poses.empty() && !global_path_.poses.empty())
  {
    // Same count but different content? Check first/last pose
    double dx0 = msg.poses.front().pose.position.x - global_path_.poses.front().pose.position.x;
    double dy0 = msg.poses.front().pose.position.y - global_path_.poses.front().pose.position.y;
    double dx1 = msg.poses.back().pose.position.x - global_path_.poses.back().pose.position.x;
    double dy1 = msg.poses.back().pose.position.y - global_path_.poses.back().pose.position.y;
    if (std::hypot(dx0, dy0) > 0.01 || std::hypot(dx1, dy1) > 0.01)
      path_changed = true;
  }

  global_path_ = msg;

  if (path_changed)
  {
    ROS_INFO("Received new /global_path (%zu poses), resetting.", msg.poses.size());
    closest_local_index_ = -1;
    end_of_avoid_index_ = -1;
    state_ = STATE::RELAYING;

    // Detect closed-loop path (first ≈ last). For loops, obstacle detection and
    // A* goal search must wrap past the endpoint back to the start, otherwise an
    // obstacle sitting exactly at the endpoint blocks avoidance forever.
    is_loop_path_ = false;
    if (msg.poses.size() > 1)
    {
      double dx = msg.poses.front().pose.position.x - msg.poses.back().pose.position.x;
      double dy = msg.poses.front().pose.position.y - msg.poses.back().pose.position.y;
      is_loop_path_ = std::hypot(dx, dy) < loop_closure_threshold_;
      if (is_loop_path_)
        ROS_INFO("Loop path detected (first≈last within %.2fm)", loop_closure_threshold_);
    }
  }

  global_path_initialized_ = true;
}

void AstarAvoid::scanCallback(const sensor_msgs::LaserScan& msg)
{
  latest_scan_ = msg;
  scan_initialized_ = true;
}

void AstarAvoid::obstacleSignalCallback(const std_msgs::Bool& msg)
{
  test_obstacle_detected_ = msg.data;
}

// ============================================================================
// Get pose from tf (map → body_foot), like test.py does
// ============================================================================

bool AstarAvoid::updatePoseFromTf()
{
  try
  {
    tf::StampedTransform transform;
    tf_listener_.lookupTransform("map", base_frame_, ros::Time(0), transform);

    current_pose_global_.header.frame_id = "map";
    current_pose_global_.header.stamp = transform.stamp_;
    current_pose_global_.pose.position.x = transform.getOrigin().x();
    current_pose_global_.pose.position.y = transform.getOrigin().y();
    current_pose_global_.pose.position.z = transform.getOrigin().z();
    current_pose_global_.pose.orientation.x = transform.getRotation().x();
    current_pose_global_.pose.orientation.y = transform.getRotation().y();
    current_pose_global_.pose.orientation.z = transform.getRotation().z();
    current_pose_global_.pose.orientation.w = transform.getRotation().w();
    // Normalize quaternion (SLAM may output slightly-off values)
    double n = std::sqrt(current_pose_global_.pose.orientation.x * current_pose_global_.pose.orientation.x +
                         current_pose_global_.pose.orientation.y * current_pose_global_.pose.orientation.y +
                         current_pose_global_.pose.orientation.z * current_pose_global_.pose.orientation.z +
                         current_pose_global_.pose.orientation.w * current_pose_global_.pose.orientation.w);
    if (n > 1e-9) {
      current_pose_global_.pose.orientation.x /= n;
      current_pose_global_.pose.orientation.y /= n;
      current_pose_global_.pose.orientation.z /= n;
      current_pose_global_.pose.orientation.w /= n;
    }
    return true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "TF error (map->%s): %s", base_frame_.c_str(), ex.what());
    return false;
  }
}

// ============================================================================
// Initialization check
// ============================================================================

bool AstarAvoid::checkInitialized()
{
  return (global_path_initialized_ && scan_initialized_);
}

// ============================================================================
// Local costmap: 5m × 4m window in map frame, centered on the car
// ============================================================================

void AstarAvoid::buildLocalCostmap()
{
  int w = static_cast<int>(local_costmap_width_  / costmap_resolution_);
  int h = static_cast<int>(local_costmap_height_ / costmap_resolution_);

  local_costmap_.header.frame_id = "map";
  local_costmap_.header.stamp    = ros::Time::now();
  local_costmap_.info.resolution = costmap_resolution_;
  local_costmap_.info.width      = w;
  local_costmap_.info.height     = h;

  // Bottom-left corner in map frame
  double cx = current_pose_global_.pose.position.x;
  double cy = current_pose_global_.pose.position.y;
  local_costmap_.info.origin.position.x = cx - local_costmap_width_  / 2.0;
  local_costmap_.info.origin.position.y = cy - local_costmap_height_ / 2.0;
  local_costmap_.info.origin.position.z = 0.0;
  local_costmap_.info.origin.orientation.w = 1.0;

  // All cells start as FREE (0)
  local_costmap_.data.assign(w * h, 0);

  // Get laser → map transform
  tf::StampedTransform laser_to_map;
  try
  {
    tf_listener_.lookupTransform("map", laser_frame_, ros::Time(0), laser_to_map);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0, "TF error (laser->map): %s", ex.what());
    return;
  }

  double ox = local_costmap_.info.origin.position.x;
  double oy = local_costmap_.info.origin.position.y;
  double res = costmap_resolution_;

  const auto& rng = latest_scan_.ranges;
  double amin = latest_scan_.angle_min;
  double ainc = latest_scan_.angle_increment;
  double rmin = latest_scan_.range_min;
  double rmax = latest_scan_.range_max;

  for (size_t i = 0; i < rng.size(); ++i)
  {
    float range = rng[i];
    if (range < rmin || range > rmax)
      continue;

    double angle = amin + i * ainc;

    // Point in laser frame
    tf::Vector3 pt_laser(range * cos(angle), range * sin(angle), 0.0);
    // Transform to map frame
    tf::Vector3 pt_map = laser_to_map * pt_laser;

    // Discretize to costmap cell
    int mx = static_cast<int>((pt_map.x() - ox) / res);
    int my = static_cast<int>((pt_map.y() - oy) / res);

    if (mx >= 0 && mx < w && my >= 0 && my < h)
    {
      int idx = my * w + mx;
      local_costmap_.data[idx] = 100;  // OCCUPIED
    }
  }

  // Clear car footprint to avoid self-collision (do this BEFORE inflation and
  // before copying the raw map, so neither the raw copy nor the inflation
  // treats the robot's own body as an obstacle)
  int car_cx = static_cast<int>((cx - ox) / res);
  int car_cy = static_cast<int>((cy - oy) / res);
  int car_r  = static_cast<int>(0.35 / res);  // covers 1.1×0.75 robot at any yaw
  for (int dx = -car_r; dx <= car_r; ++dx)
  {
    for (int dy = -car_r; dy <= car_r; ++dy)
    {
      int ci = car_cx + dx;
      int cj = car_cy + dy;
      if (ci >= 0 && ci < w && cj >= 0 && cj < h)
        local_costmap_.data[cj * w + ci] = 0;
    }
  }

  // ---- Raw (un-inflated) costmap copy ----
  // This is what isObstacleOnPath() checks during AVOIDING to detect NEW
  // obstacles on the detour. The inflated map must NOT be used for that check:
  // the A* detour legitimately runs close to the inflated boundary, so checking
  // the inflated map would constantly re-trigger replanning → stop-and-go.
  local_costmap_raw_ = local_costmap_;

  // ---- Obstacle inflation ----
  // Expand each occupied cell by obstacle_inflation_radius_ so that A* plans
  // a path with enough clearance for the robot body (1.1m×0.75m robot).
  int inflate_cells = static_cast<int>(obstacle_inflation_radius_ / res) + 1;
  int inflate_sq = inflate_cells * inflate_cells;

  // Copy raw laser hits before inflation (in-place inflation would chain-expand)
  std::vector<int8_t> raw_hits = local_costmap_.data;

  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      if (raw_hits[y * w + x] < 50)
        continue;  // not an occupied cell

      // Inflate a circular region around this occupied cell
      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy)
      {
        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx)
        {
          if (dx * dx + dy * dy > inflate_sq)
            continue;  // outside the inflation circle

          int nx = x + dx;
          int ny = y + dy;
          if (nx >= 0 && nx < w && ny >= 0 && ny < h)
            local_costmap_.data[ny * w + nx] = 100;
        }
      }
    }
  }
}

// ============================================================================
// Check if a point (map frame) lies inside the current local costmap
// ============================================================================

bool AstarAvoid::isInsideCostmap(const geometry_msgs::Point& p)
{
  if (local_costmap_.data.empty())
    return false;

  double ox = local_costmap_.info.origin.position.x;
  double oy = local_costmap_.info.origin.position.y;
  double res = costmap_resolution_;
  int w = local_costmap_.info.width;
  int h = local_costmap_.info.height;

  int mx = static_cast<int>((p.x - ox) / res);
  int my = static_cast<int>((p.y - oy) / res);

  return (mx >= 0 && mx < w && my >= 0 && my < h);
}

// ============================================================================
// Obstacle-on-path detection
// Checks if any upcoming path point is near an occupied cell of the given
// costmap. Pass local_costmap_raw_ (un-inflated) during AVOIDING so the A*
// detour — which legitimately hugs the inflated boundary — does not constantly
// re-trigger replanning. Only NEW, real obstacles on the detour are detected.
// (path is the currently-tracked path: global_path_ or avoid_waypoints_)
// ============================================================================

bool AstarAvoid::isObstacleOnPath(const nav_msgs::Path& path, const nav_msgs::OccupancyGrid& costmap)
{
  if (path.poses.empty() || costmap.data.empty())
    return false;

  int closest_idx = getClosestWaypoint(path, current_pose_global_.pose);
  if (closest_idx < 0)
    return false;

  int w = costmap.info.width;
  int h = costmap.info.height;
  double ox = costmap.info.origin.position.x;
  double oy = costmap.info.origin.position.y;
  double res = costmap_resolution_;
  int search_r = static_cast<int>(obstacle_on_path_threshold_ / res) + 1;

  // Car position and forward direction (for ignoring obstacles behind the car)
  double car_x = current_pose_global_.pose.position.x;
  double car_y = current_pose_global_.pose.position.y;
  double car_yaw = tf::getYaw(current_pose_global_.pose.orientation);
  double car_fwd_x = std::cos(car_yaw);
  double car_fwd_y = std::sin(car_yaw);

  double accumulated_dist = 0.0;
  int n = static_cast<int>(path.poses.size());

  for (int i = closest_idx; i < n - 1; ++i)
  {
    double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    accumulated_dist += std::hypot(dx, dy);

    if (accumulated_dist > obstacle_check_distance_)
      break;

    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;

    int mx = static_cast<int>((px - ox) / res);
    int my = static_cast<int>((py - oy) / res);

    for (int dcy = -search_r; dcy <= search_r; ++dcy)
    {
      for (int dcx = -search_r; dcx <= search_r; ++dcx)
      {
        int cx = mx + dcx;
        int cy = my + dcy;
        if (cx >= 0 && cx < w && cy >= 0 && cy < h)
        {
          if (costmap.data[cy * w + cx] >= 50)
          {
            // Only care about obstacles in front half-plane of the car
            double obs_x = ox + cx * res;
            double obs_y = oy + cy * res;
            double to_obs_x = obs_x - car_x;
            double to_obs_y = obs_y - car_y;
            double dot = to_obs_x * car_fwd_x + to_obs_y * car_fwd_y;
            if (dot < 0.0)
              continue;  // obstacle behind car, ignore

            ROS_INFO_THROTTLE(1.0, "Obstacle on path at idx %d, dist=%.2fm", i, accumulated_dist);
            return true;
          }
        }
      }
    }
  }

  return false;
}

// ============================================================================
// Find a point on global_path beyond the obstacle for A* goal
// Returns:
//   >= 0 : valid goal index on global path
//   -1   : obstacle is gone, no goal needed
//   -2   : obstacle present but no in-costmap goal found → cannot avoid locally,
//          vehicle must stay stopped safely
// ============================================================================

int AstarAvoid::findGoalBeyondObstacle()
{
  if (local_costmap_.data.empty())
    return -1;

  // Thread-safe snapshot of global path (globalPathCallback writes it)
  nav_msgs::Path path;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    path = global_path_;
  }

  if (path.poses.empty())
    return -1;

  int closest_idx = getClosestWaypoint(path, current_pose_global_.pose);
  if (closest_idx < 0)
    return -1;

  double ox = local_costmap_.info.origin.position.x;
  double oy = local_costmap_.info.origin.position.y;
  double res = costmap_resolution_;
  int w = local_costmap_.info.width;
  int h = local_costmap_.info.height;

  int n = static_cast<int>(path.poses.size());
  bool passed_obstacle = false;

  // Two search radii:
  //  - small (obstacle_on_path_threshold_): for detecting obstacle presence on path
  //  - large (goal_clearance_): must cover robot half-diagonal (~0.67m for 1.1x0.75),
  //    ensures goal point is truly collision-free for detectCollision()
  int search_r_obstacle = static_cast<int>(obstacle_on_path_threshold_ / res) + 1;
  double goal_clearance = 0.35;  // half-diagonal + margin
  int search_r_goal = static_cast<int>(goal_clearance / res) + 1;

  // For loop paths the path continues past the last point back to the first, so
  // the search (and the goal lookahead) must wrap around the endpoint. Without
  // this, an obstacle sitting exactly at the endpoint leaves no clean rejoin
  // point past it → A* goal is the endpoint itself (in collision) → A* fails →
  // the robot just stops. Wrapping lets us find the rejoin point on the segment
  // after the loop closes, so the robot keeps looping and avoiding.
  int max_iter = is_loop_path_ ? n : (n - closest_idx);

  for (int k = 0; k < max_iter; ++k)
  {
    int i = is_loop_path_ ? (closest_idx + k) % n : (closest_idx + k);

    // Check if this path point is near an obstacle
    double px = path.poses[i].pose.position.x;
    double py = path.poses[i].pose.position.y;
    int mx = static_cast<int>((px - ox) / res);
    int my = static_cast<int>((py - oy) / res);

    // Lambda: check if any occupied cell within search_r of (mx, my)
    auto hasObstacleNear = [&](int r) -> bool {
      for (int dcy = -r; dcy <= r; ++dcy)
      {
        for (int dcx = -r; dcx <= r; ++dcx)
        {
          int cx = mx + dcx;
          int cy = my + dcy;
          if (cx >= 0 && cx < w && cy >= 0 && cy < h)
          {
            if (local_costmap_.data[cy * w + cx] >= 50)
              return true;
          }
        }
      }
      return false;
    };

    if (!passed_obstacle)
    {
      // Still before obstacle: use small radius to detect obstacle presence
      if (hasObstacleNear(search_r_obstacle))
      {
        passed_obstacle = true;
      }
    }
    else
    {
      // Past obstacle: first check with small radius (still in obstacle zone?)
      if (hasObstacleNear(search_r_obstacle))
        continue;  // still near obstacle, keep walking

      // Small check passed → verify with large radius (robot footprint)
      if (hasObstacleNear(search_r_goal))
        continue;  // small was clean but large detects collision → keep walking

      // Truly clean for robot footprint! Skip ahead by lookahead distance
      int goal_i = i;
      double lookahead = 0.0;
      int la_iter = is_loop_path_ ? n : (n - i - 1);
      for (int j = 1; j <= la_iter; ++j)
      {
        int ji = is_loop_path_ ? (i + j) % n : (i + j);
        int jp = is_loop_path_ ? (i + j - 1) % n : (i + j - 1);
        double dx = path.poses[ji].pose.position.x - path.poses[jp].pose.position.x;
        double dy = path.poses[ji].pose.position.y - path.poses[jp].pose.position.y;
        lookahead += std::hypot(dx, dy);
        goal_i = ji;
        if (lookahead >= a_star_goal_lookahead_)
          break;
      }

      // The A* goal must be inside the local costmap, otherwise we cannot
      // plan a local detour → keep the vehicle stopped safely.
      if (!isInsideCostmap(path.poses[goal_i].pose.position))
      {
        ROS_WARN_THROTTLE(1.0, "A* goal idx=%d outside local costmap → cannot avoid locally", goal_i);
        return -2;
      }

      last_goal_idx_ = goal_i;
      ROS_INFO("A* goal: idx=%d (first_clean=%d, lookahead=%.2fm)", goal_i, i, lookahead);
      return goal_i;
    }
  }

  // No obstacle found at all on the path → obstacle is gone
  if (!passed_obstacle)
  {
    ROS_INFO("A* goal: no obstacle detected on path → cleared");
    return -1;  // signal "obstacle gone" to PLANNING state
  }

  // Obstacle found but no clean point past it → fallback to last point,
  // but only if it is reachable within the local costmap.
  last_goal_idx_ = n - 1;
  if (!isInsideCostmap(path.poses[last_goal_idx_].pose.position))
  {
    ROS_WARN_THROTTLE(1.0, "A* goal: no clean point, last idx=%d outside costmap → cannot avoid locally", last_goal_idx_);
    return -2;
  }
  ROS_WARN_THROTTLE(1.0, "A* goal: obstacle present, no clean point past it, using last idx=%d", last_goal_idx_);
  return last_goal_idx_;
}

// ============================================================================
// Build a "distance to reference path" map (costmap frame) used to keep the
// A* detour hugging the global path. Without it, A* can take a straight-line
// shortcut through free space (e.g. across the middle of a rounded-rectangle
// loop) instead of following the path around the obstacle.
//
// Only a forward window of the path (a few meters ahead of the car) plus a
// short tail behind is used, so for loop paths the FAR side of the loop is NOT
// counted as "near the path" — otherwise a narrow loop would let A* cut across
// to the other side.
// ============================================================================

void AstarAvoid::computePathDeviationMap(const nav_msgs::Path& path,
                                         std::vector<double>& dev, int w, int h)
{
  dev.assign(w * h, 0.0);
  if (path.poses.empty())
    return;

  double ox = local_costmap_.info.origin.position.x;
  double oy = local_costmap_.info.origin.position.y;
  double res = costmap_resolution_;
  int n = static_cast<int>(path.poses.size());

  int closest_idx = getClosestWaypoint(path, current_pose_global_.pose);
  if (closest_idx < 0)
    return;

  // Reference segment: forward window along the path (~3m) + short tail behind
  // the car (~1m) so the area right at the start is covered too.
  std::vector<std::pair<double, double>> pts;
  double dist = 0.0;
  for (int s = 0; s < n; ++s)
  {
    int idx = is_loop_path_ ? (closest_idx + s) % n : (closest_idx + s);
    if (!is_loop_path_ && idx >= n)
      break;
    if (s > 0)
    {
      int prev = is_loop_path_ ? (closest_idx + s - 1) % n : (closest_idx + s - 1);
      dist += std::hypot(path.poses[idx].pose.position.x - path.poses[prev].pose.position.x,
                         path.poses[idx].pose.position.y - path.poses[prev].pose.position.y);
      if (dist > 3.0)
        break;
    }
    pts.emplace_back(path.poses[idx].pose.position.x, path.poses[idx].pose.position.y);
  }
  dist = 0.0;
  for (int s = 1; s < n; ++s)
  {
    int idx = is_loop_path_ ? (closest_idx - s + n) % n : (closest_idx - s);
    if (!is_loop_path_ && idx < 0)
      break;
    int prev = is_loop_path_ ? (closest_idx - s + 1 + n) % n : (closest_idx - s + 1);
    dist += std::hypot(path.poses[idx].pose.position.x - path.poses[prev].pose.position.x,
                       path.poses[idx].pose.position.y - path.poses[prev].pose.position.y);
    if (dist > 1.0)
      break;
    pts.emplace_back(path.poses[idx].pose.position.x, path.poses[idx].pose.position.y);
  }

  if (pts.empty())
    return;

  for (int my = 0; my < h; ++my)
  {
    for (int mx = 0; mx < w; ++mx)
    {
      double cx = ox + (mx + 0.5) * res;
      double cy = oy + (my + 0.5) * res;
      double best = std::numeric_limits<double>::max();
      for (const auto& p : pts)
      {
        double dx = p.first - cx;
        double dy = p.second - cy;
        double d2 = dx * dx + dy * dy;
        if (d2 < best)
          best = d2;
      }
      dev[my * w + mx] = std::sqrt(best);
    }
  }
}

// ============================================================================
// A* planning: plan from current pose to a point beyond the obstacle
// ============================================================================

bool AstarAvoid::planAvoidWaypoints(const int& goal_idx)
{
  if (goal_idx < 0)
  {
    ROS_WARN("Cannot determine A* goal.");
    return false;
  }

  geometry_msgs::PoseStamped goal_global = global_path_.poses[goal_idx];

  // Convert poses to costmap frame
  tf::Transform map_to_costmap;
  tf::poseMsgToTF(local_costmap_.info.origin, map_to_costmap);
  map_to_costmap = map_to_costmap.inverse();

  tf::Pose tf_pose;
  tf::poseMsgToTF(current_pose_global_.pose, tf_pose);
  tf_pose = map_to_costmap * tf_pose;
  geometry_msgs::Pose start_local;
  tf::poseTFToMsg(tf_pose, start_local);

  tf::poseMsgToTF(goal_global.pose, tf_pose);
  tf_pose = map_to_costmap * tf_pose;
  geometry_msgs::Pose goal_local;
  tf::poseTFToMsg(tf_pose, goal_local);

  // Initialize A* with the local costmap.
  // Also feed a path-deviation map so the detour stays close to the global path
  // instead of cutting across free space (e.g. through the loop's interior).
  std::vector<double> dev_map;
  computePathDeviationMap(global_path_, dev_map,
                          local_costmap_.info.width, local_costmap_.info.height);
  astar_.setPathDeviationMap(dev_map, local_costmap_.info.width, local_costmap_.info.height);

  astar_.initialize(local_costmap_);

  // Run A*
  bool found = astar_.makePlan(start_local, goal_local);

  if (found)
  {
    ROS_INFO("A* success: %zu poses.", astar_.getPath().poses.size());
    astar_path_pub_.publish(astar_.getPath());
    mergeAvoidWaypoints(astar_.getPath());
  }
  else
  {
    double map_w = local_costmap_.info.width * local_costmap_.info.resolution;
    double map_h = local_costmap_.info.height * local_costmap_.info.resolution;
    ROS_WARN("A* FAILED | map=%.1fx%.1fm start=(%.2f,%.2f) goal=(%.2f,%.2f) idx=%d",
             map_w, map_h, start_local.position.x, start_local.position.y,
             goal_local.position.x, goal_local.position.y, goal_idx);
  }

  astar_.reset();
  return found;
}

// ============================================================================
// Merge A* path with original global_path
// ============================================================================

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& a_star_path)
{
  std::lock_guard<std::mutex> lock(mutex_);

  avoid_waypoints_.poses.clear();
  avoid_waypoints_.header = global_path_.header;

  // New avoidance path → invalidate the cached closest index (it belonged to
  // the previous path; without this, repeated avoidances on a closed loop can
  // read out of range in getLocalClosestWaypoint() and crash)
  closest_local_index_ = -1;

  int closest_idx = getClosestWaypoint(global_path_, current_pose_global_.pose);
  if (closest_idx < 0)
  {
    avoid_waypoints_ = a_star_path;
    return;
  }

  // Copy waypoints before car
  for (int i = 0; i < closest_idx; ++i)
    avoid_waypoints_.poses.push_back(global_path_.poses[i]);

  // Insert A* path (convert from costmap frame to map frame)
  tf::Transform costmap_to_map;
  tf::poseMsgToTF(local_costmap_.info.origin, costmap_to_map);

  for (const auto& ps : a_star_path.poses)
  {
    geometry_msgs::PoseStamped wp;
    wp.header = avoid_waypoints_.header;
    tf::Pose tp;
    tf::poseMsgToTF(ps.pose, tp);
    tp = costmap_to_map * tp;
    tf::poseTFToMsg(tp, wp.pose);
    wp.pose.position.z = current_pose_global_.pose.position.z;
    avoid_waypoints_.poses.push_back(wp);
  }

  // Append global path after obstacle (use cached goal index from planAvoidWaypoints)
  if (last_goal_idx_ >= 0 && last_goal_idx_ < static_cast<int>(global_path_.poses.size()))
  {
    for (int i = last_goal_idx_; i < static_cast<int>(global_path_.poses.size()); ++i)
      avoid_waypoints_.poses.push_back(global_path_.poses[i]);
  }

  end_of_avoid_index_ = closest_idx + static_cast<int>(a_star_path.poses.size());
  ROS_INFO("Merged: %zu poses, rejoin at %d",
           avoid_waypoints_.poses.size(), last_goal_idx_);
}

// ============================================================================
// Main run loop
// ============================================================================

void AstarAvoid::run()
{
  state_ = AstarAvoid::STATE::INITIALIZING;

  while (ros::ok())
  {
    ros::spinOnce();
    if (checkInitialized())
      break;
    ROS_WARN_THROTTLE(2.0, "Waiting for topics...");
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("AstarAvoid running. Obstacle detection: /obstacle_stop from test.py");

  state_ = AstarAvoid::STATE::RELAYING;
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  publish_thread_ = std::thread(&AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();

    // Get current pose from tf (map → body_foot)
    if (!updatePoseFromTf())
    {
      rate_->sleep();
      continue;
    }

    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // Build local costmap every cycle
    buildLocalCostmap();
    local_costmap_pub_.publish(local_costmap_);

    // Obstacle-on-path detection:
    //  - AVOIDING : use the local costmap to check the avoidance path itself
    //               (a NEW obstacle on the detour path/goal → trigger replan)
    //  - otherwise: rely ONLY on test.py's /obstacle_stop signal to trigger
    //               avoidance (isObstacleOnPath() is not used outside AVOIDING)
    bool obstacle_on_path = test_obstacle_detected_;
    if (state_ == AstarAvoid::STATE::AVOIDING)
    {
      nav_msgs::Path path_for_check;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        path_for_check = avoid_waypoints_;
      }
      // Use the RAW (un-inflated) costmap here: the detour intentionally runs
      // close to the inflated boundary, so checking the inflated map would make
      // the robot constantly think there is an obstacle on the detour and keep
      // stopping to replan (stop-and-go). Only genuinely new obstacles on the
      // detour should trigger a replan.
      const nav_msgs::OccupancyGrid& on_path_map =
          local_costmap_raw_.data.empty() ? local_costmap_ : local_costmap_raw_;
      obstacle_on_path = isObstacleOnPath(path_for_check, on_path_map);
    }

    // State feedback
    std_msgs::Int32 state_msg;
    state_msg.data = static_cast<int>(state_);
    avoidance_state_pub_.publish(state_msg);

    // ---- State machine ----
    switch (state_)
    {
      case AstarAvoid::STATE::RELAYING:
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          avoid_waypoints_ = global_path_;
        }
        if (obstacle_on_path)
        {
          ROS_INFO("[RELAYING -> STOPPING] obstacle on path");
          state_ = AstarAvoid::STATE::STOPPING;
          start_plan_time = ros::WallTime::now();  // reset timer on entry
        }
        break;
      }

      case AstarAvoid::STATE::STOPPING:
      {
        bool timeout = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);
        if (timeout)
        {
          // Always enter PLANNING after timeout — PLANNING will check
          // if obstacle is truly still there via findGoalBeyondObstacle()
          ROS_INFO("[STOPPING -> PLANNING] timeout, checking A*");
          state_ = AstarAvoid::STATE::PLANNING;
        }
        break;
      }

      case AstarAvoid::STATE::PLANNING:
      {
        start_plan_time = ros::WallTime::now();
        // Quick check: is obstacle still there?
        int goal_idx = findGoalBeyondObstacle();
        if (goal_idx == -1)
        {
          ROS_INFO("[PLANNING -> RELAYING] obstacle gone, no goal needed");
          state_ = AstarAvoid::STATE::RELAYING;
        }
        else if (goal_idx == -2)
        {
          ROS_WARN("[PLANNING -> STOPPING] cannot avoid locally (goal out of costmap), stay stopped");
          state_ = AstarAvoid::STATE::STOPPING;
          avoid_waypoints_.poses.clear();
        }
        else if (planAvoidWaypoints(goal_idx))
        {
          ROS_INFO("[PLANNING -> AVOIDING] A* success");
          state_ = AstarAvoid::STATE::AVOIDING;
          start_avoid_time = ros::WallTime::now();
        }
        else
        {
          ROS_WARN("[PLANNING -> STOPPING] A* failed, fallback stop");
          state_ = AstarAvoid::STATE::STOPPING;
          avoid_waypoints_.poses.clear();
        }
        break;
      }

      case AstarAvoid::STATE::AVOIDING:
      {
        bool reached = (getLocalClosestWaypoint(avoid_waypoints_, current_pose_global_.pose, closest_search_size_)
                        >= end_of_avoid_index_);
        if (reached)
        {
          ROS_INFO("[AVOIDING -> RELAYING] rejoined path");
          state_ = AstarAvoid::STATE::RELAYING;
        }
        else if (obstacle_on_path)
        {
          bool timeout = ((ros::WallTime::now() - start_avoid_time).toSec() > replan_interval_);
          if (timeout)
          {
            ROS_INFO("[AVOIDING -> STOPPING] new obstacle on detour path");
            state_ = AstarAvoid::STATE::STOPPING;
            start_plan_time = ros::WallTime::now();  // hold stopped before replanning
          }
        }
        break;
      }

      default:
        break;
    }

    rate_->sleep();
  }

  terminate_thread_ = true;
}

// ============================================================================
// Path publishing (separate thread)
// ============================================================================

void AstarAvoid::publishWaypoints()
{
  nav_msgs::Path current_waypoints;

  while (!terminate_thread_)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      switch (state_)
      {
        case AstarAvoid::STATE::RELAYING:
          current_waypoints = global_path_;
          break;
        case AstarAvoid::STATE::STOPPING:
          // Publish global_path → test.py handles obstacle braking
          current_waypoints = global_path_;
          break;
        case AstarAvoid::STATE::PLANNING:
          break;  // keep previous
        case AstarAvoid::STATE::AVOIDING:
          current_waypoints = avoid_waypoints_;
          break;
        default:
          current_waypoints = global_path_;
          break;
      }

      safety_waypoints_.poses.clear();
      safety_waypoints_.header = current_waypoints.header;

      if (!current_waypoints.poses.empty())
      {
        // Publish the full path — test.py handles nearest-point lookup on its own,
        // so already-passed points don't interfere. Full path also enables loop
        // detection (first≈last) in test.py for closed-loop tracking.
        safety_waypoints_.poses = current_waypoints.poses;
      }
    }  // mutex_ released here (before publish to avoid holding lock during serialization)

    safety_waypoints_pub_.publish(safety_waypoints_);

    rate_->sleep();
  }
}

// ============================================================================
// Utility functions
// ============================================================================

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_THROTTLE(5.0, "TF error (%s->%s): %s", from.c_str(), to.c_str(), ex.what());
  }
  return stf;
}

int AstarAvoid::getClosestWaypoint(const nav_msgs::Path& waypoints, const geometry_msgs::Pose& pose)
{
  if (waypoints.poses.empty())
    return -1;

  int closest_idx = -1;
  double closest_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < waypoints.poses.size(); ++i)
  {
    double dx = waypoints.poses[i].pose.position.x - pose.position.x;
    double dy = waypoints.poses[i].pose.position.y - pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < closest_dist)
    {
      closest_dist = dist;
      closest_idx = static_cast<int>(i);
    }
  }

  return closest_idx;
}

int AstarAvoid::getLocalClosestWaypoint(const nav_msgs::Path& waypoints, const geometry_msgs::Pose& pose, const int& search_size)
{
  static nav_msgs::Path local_waypoints;
  const int prev_idx = closest_local_index_;

  if (closest_local_index_ == -1)
  {
    closest_local_index_ = getClosestWaypoint(waypoints, pose);
  }
  else
  {
    int n = static_cast<int>(waypoints.poses.size());
    int start = std::max(0, prev_idx - search_size / 2);
    int end   = std::min(prev_idx + search_size / 2, n);
    local_waypoints.poses.assign(waypoints.poses.begin() + start, waypoints.poses.begin() + end);
    closest_local_index_ = start + getClosestWaypoint(local_waypoints, pose);
  }

  return closest_local_index_;
}
