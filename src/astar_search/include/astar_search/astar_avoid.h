/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ASTAR_AVOID_H
#define ASTAR_AVOID_H

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "astar_search/astar_search.h"

class AstarAvoid
{
public:
  typedef enum STATE
  {
    INITIALIZING = -1,
    RELAYING = 0,
    STOPPING = 1,
    PLANNING = 2,
    AVOIDING = 3
  } State;

  AstarAvoid();
  ~AstarAvoid();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher safety_waypoints_pub_;
  ros::Publisher local_costmap_pub_;      // debug: publish local costmap for rviz
  ros::Publisher astar_path_pub_;         // debug: raw A* planned path
  ros::Publisher avoidance_state_pub_;    // feedback: current avoidance state
  ros::Subscriber global_path_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber obstacle_signal_sub_;    // /obstacle_stop from test.py
  ros::Rate *rate_;
  tf::TransformListener tf_listener_;

  // params - local costmap (5m x 4m window, follows the car)
  double local_costmap_width_;      // [m], default 5.0
  double local_costmap_height_;     // [m], default 4.0
  double costmap_resolution_;       // [m], default 0.05
  std::string laser_frame_;         // laser scan tf frame, default "body"
  std::string base_frame_;          // robot base tf frame, default "body_foot"

  // params - obstacle detection
  double obstacle_check_distance_;       // look ahead distance along path [m], default 3.0
  double obstacle_on_path_threshold_;    // distance threshold for "obstacle on path" [m], default 0.3
  double obstacle_inflation_radius_;     // inflation radius around each laser hit [m], default 0.2
  double loop_closure_threshold_;        // first≈last distance below which the path is a loop [m]

  // params - avoidance
  bool enable_avoidance_;                // enable avoidance mode
  double avoid_start_velocity_;          // max velocity to start replanning [km/h]
  double replan_interval_;               // min interval between replan attempts [s]
  double a_star_goal_lookahead_;         // how far past obstacle to set A* goal [m], default 2.0
  int safety_waypoints_size_;            // output waypoint count [-]
  double update_rate_;                   // publishing rate [Hz]
  int closest_search_size_;              // search closest waypoint window [-]

  // classes
  AstarSearch astar_;
  State state_;

  // threads
  std::thread publish_thread_;
  std::mutex mutex_;

  // variables
  bool terminate_thread_;
  int closest_local_index_;
  int end_of_avoid_index_;
  int last_goal_idx_;                       // cached A* goal index for merge
  nav_msgs::OccupancyGrid local_costmap_;    // 5m x 4m local costmap in map frame (inflated, used for A*)
  nav_msgs::OccupancyGrid local_costmap_raw_;  // raw (un-inflated) copy; used for on-path check during AVOIDING
  nav_msgs::Path global_path_;               // recorded trajectory from /global_path
  nav_msgs::Path avoid_waypoints_;           // A*-planned avoiding path
  nav_msgs::Path safety_waypoints_;          // output path to controller
  geometry_msgs::PoseStamped current_pose_global_;
  geometry_msgs::PoseStamped current_pose_local_;  // pose in costmap frame
  sensor_msgs::LaserScan latest_scan_;       // latest laser scan

  bool global_path_initialized_;
  bool scan_initialized_;
  bool test_obstacle_detected_;     // signal from test.py's obstacle detection
  bool is_loop_path_;               // global path is a closed loop (first ≈ last)

  // callbacks
  void globalPathCallback(const nav_msgs::Path& msg);
  void scanCallback(const sensor_msgs::LaserScan& msg);
  void obstacleSignalCallback(const std_msgs::Bool& msg);

  // core functions
  bool updatePoseFromTf();
  bool checkInitialized();
  void buildLocalCostmap();
  bool isInsideCostmap(const geometry_msgs::Point& p);
  bool isObstacleOnPath(const nav_msgs::Path& path, const nav_msgs::OccupancyGrid& costmap);
  int findGoalBeyondObstacle();
  void computePathDeviationMap(const nav_msgs::Path& path, std::vector<double>& dev, int w, int h);
  bool planAvoidWaypoints(const int& goal_idx);
  void mergeAvoidWaypoints(const nav_msgs::Path& a_star_path);
  void publishWaypoints();

  // utility functions
  tf::Transform getTransform(const std::string& from, const std::string& to);
  int getClosestWaypoint(const nav_msgs::Path& waypoints, const geometry_msgs::Pose& pose);
  int getLocalClosestWaypoint(const nav_msgs::Path& waypoints, const geometry_msgs::Pose& pose, const int& search_size);
};

#endif
