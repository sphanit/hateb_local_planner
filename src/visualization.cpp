/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Copyright (c) 2016 LAAS/CNRS
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann
 *          Harmish Khambhaita (harmish@laas.fr)
 *********************************************************************/

#define GLOBAL_PLAN_TOPIC "global_plan"
#define LOCAL_PLAN_TOPIC "local_plan"
#define LOCAL_PLAN_POSES_TOPIC "local_plan_poses"
#define HUMAN_GLOBAL_PLANS_TOPIC "human_global_plans"
#define HUMAN_LOCAL_PLANS_TOPIC "human_local_plans"
#define HUMAN_LOCAL_PLAN_POSES_TOPIC "human_local_plan_poses"
#define GLOBAL_PLAN_TOPIC "global_plan"
#define CLEARING_TIMER_DURATION 1.0 // seconds

#include <teb_local_planner/visualization.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/FeedbackMsg.h>

namespace teb_local_planner {

TebVisualization::TebVisualization() : initialized_(false) {}

TebVisualization::TebVisualization(ros::NodeHandle &nh, const TebConfig &cfg)
    : initialized_(false) {
  initialize(nh, cfg);
}

void TebVisualization::initialize(ros::NodeHandle &nh, const TebConfig &cfg) {
  if (initialized_)
    ROS_WARN("TebVisualization already initialized. Reinitalizing...");

  // set config
  cfg_ = &cfg;

  // register topics
  global_plan_pub_ = nh.advertise<nav_msgs::Path>(GLOBAL_PLAN_TOPIC, 1);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>(LOCAL_PLAN_TOPIC, 1);
  teb_poses_pub_ =
      nh.advertise<geometry_msgs::PoseArray>(LOCAL_PLAN_POSES_TOPIC, 1);
  humans_global_plans_pub_ =
      nh.advertise<hanp_msgs::HumanPathArray>(HUMAN_GLOBAL_PLANS_TOPIC, 1);
  humans_local_plans_pub_ =
      nh.advertise<hanp_msgs::HumanTrajectoryArray>(HUMAN_LOCAL_PLANS_TOPIC, 1);
  humans_tebs_poses_pub_ =
      nh.advertise<geometry_msgs::PoseArray>(HUMAN_LOCAL_PLAN_POSES_TOPIC, 1);
  teb_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("teb_markers", 1000);
  feedback_pub_ =
      nh.advertise<teb_local_planner::FeedbackMsg>("teb_feedback", 10);

  last_publish_robot_global_plan =
      cfg_->visualization.publish_robot_global_plan;
  last_publish_robot_local_plan = cfg_->visualization.publish_robot_local_plan;
  last_publish_robot_local_plan_poses =
      cfg_->visualization.publish_robot_local_plan_poses;
  last_publish_human_global_plans =
      cfg_->visualization.publish_human_global_plans;
  last_publish_human_local_plans =
      cfg_->visualization.publish_human_local_plans;
  last_publish_human_local_plan_poses =
      cfg_->visualization.publish_human_local_plan_poses;
  clearing_timer_ = nh.createTimer(ros::Duration(CLEARING_TIMER_DURATION),
                                   &TebVisualization::clearingTimerCB, this);

  initialized_ = true;
}

void TebVisualization::publishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped> &global_plan) const {
  if (printErrorWhenNotInitialized() ||
      !cfg_->visualization.publish_robot_global_plan) {
    return;
  }
  base_local_planner::publishPlan(global_plan, global_plan_pub_);
}

void TebVisualization::publishLocalPlan(
    const std::vector<geometry_msgs::PoseStamped> &local_plan) const {
  if (printErrorWhenNotInitialized())
    return;
  base_local_planner::publishPlan(local_plan, local_plan_pub_);
}

void TebVisualization::publishHumansPlans(
    const std::vector<HumanPlanCombined> &humans_plans) const {
  if (printErrorWhenNotInitialized() ||
      !cfg_->visualization.publish_human_global_plans || humans_plans.empty()) {
    return;
  }

  auto now = ros::Time::now();
  auto frame_id = cfg_->map_frame;

  hanp_msgs::HumanPathArray human_path_array;
  human_path_array.header.stamp = now;
  human_path_array.header.frame_id = frame_id;

  for (auto &human_plan_combined : humans_plans) {
    auto total_size = human_plan_combined.plan_before.size() +
                      human_plan_combined.plan_to_optimize.size() +
                      human_plan_combined.plan_after.size();
    if (total_size == 0) {
      continue;
    }

    nav_msgs::Path path;
    path.header.stamp = now;
    path.header.frame_id = frame_id;

    path.poses.resize(total_size);
    size_t index = 0;
    for (size_t i = 0; i < human_plan_combined.plan_before.size(); ++i) {
      path.poses[i] = human_plan_combined.plan_before[i];
    }
    index += human_plan_combined.plan_before.size();
    for (size_t i = 0; i < human_plan_combined.plan_to_optimize.size(); ++i) {
      path.poses[i + index] = human_plan_combined.plan_to_optimize[i];
    }
    index += human_plan_combined.plan_to_optimize.size();
    for (size_t i = 0; i < human_plan_combined.plan_after.size(); ++i) {
      path.poses[i + index] = human_plan_combined.plan_after[i];
    }

    hanp_msgs::HumanPath human_path;
    human_path.header.stamp = now;
    human_path.header.frame_id = frame_id;
    human_path.id = human_plan_combined.id;
    human_path.path = path;

    human_path_array.paths.push_back(human_path);
  }

  if (!human_path_array.paths.empty()) {
    humans_global_plans_pub_.publish(human_path_array);
  }
}

void TebVisualization::publishLocalPlanAndPoses(
    const TimedElasticBand &teb) const {
  if (printErrorWhenNotInitialized() ||
      (!cfg_->visualization.publish_robot_local_plan &&
       !cfg_->visualization.publish_robot_local_plan_poses)) {
    return;
  }

  auto frame_id = cfg_->map_frame;
  auto now = ros::Time::now();

  // create path msg
  nav_msgs::Path teb_path;
  teb_path.header.frame_id = frame_id;
  teb_path.header.stamp = now;

  // create pose_array (along trajectory)
  geometry_msgs::PoseArray teb_poses;
  teb_poses.header.frame_id = frame_id;
  teb_poses.header.stamp = now;

  // fill path msgs with teb configurations
  double pose_time = 0.0;
  for (unsigned int i = 0; i < teb.sizePoses(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = now;
    pose.pose.position.x = teb.Pose(i).x();
    pose.pose.position.y = teb.Pose(i).y();
    pose.pose.position.z = 0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
    teb_path.poses.push_back(pose);
    pose.pose.position.z = pose_time * cfg_->visualization.pose_array_z_scale;
    teb_poses.poses.push_back(pose.pose);
    if (i < (teb.sizePoses() - 1)) {
      pose_time += teb.TimeDiff(i);
    }
  }
  if (!teb_path.poses.empty() && cfg_->visualization.publish_robot_local_plan) {
    local_plan_pub_.publish(teb_path);
  }
  if (!teb_poses.poses.empty() &&
      cfg_->visualization.publish_robot_local_plan_poses) {
    teb_poses_pub_.publish(teb_poses);
  }
}

void TebVisualization::publishHumanPlanPoses(
    const std::map<uint64_t, TimedElasticBand> &humans_tebs_map) const {
  if (printErrorWhenNotInitialized() ||
      !cfg_->visualization.publish_human_local_plan_poses ||
      humans_tebs_map.empty()) {
    return;
  }

  geometry_msgs::PoseArray gui_teb_poses;
  gui_teb_poses.header.frame_id = cfg_->map_frame;
  gui_teb_poses.header.stamp = ros::Time::now();

  for (auto &human_teb_kv : humans_tebs_map) {
    auto &human_id = human_teb_kv.first;
    auto &human_teb = human_teb_kv.second;

    if (human_teb.sizePoses() == 0)
      continue;

    double pose_time = 0;
    for (unsigned int i = 0; i < human_teb.sizePoses(); i++) {
      geometry_msgs::Pose pose;
      pose.position.x = human_teb.Pose(i).x();
      pose.position.y = human_teb.Pose(i).y();
      pose.position.z = pose_time * cfg_->visualization.pose_array_z_scale;
      pose.orientation =
          tf::createQuaternionMsgFromYaw(human_teb.Pose(i).theta());
      gui_teb_poses.poses.push_back(pose);
      if (i < (human_teb.sizePoses() - 1)) {
        pose_time += human_teb.TimeDiff(i);
      }
    }
  }

  if (!gui_teb_poses.poses.empty()) {
    humans_tebs_poses_pub_.publish(gui_teb_poses);
  }
}

void TebVisualization::publishHumanTrajectories(
    const std::vector<HumanPlanTrajCombined> &humans_plans_traj_combined)
    const {
  if (printErrorWhenNotInitialized() ||
      !cfg_->visualization.publish_human_local_plans) {
    return;
  }

  auto now = ros::Time::now();
  auto frame_id = cfg_->map_frame;

  hanp_msgs::HumanTrajectoryArray hanp_trajectory_array;
  hanp_trajectory_array.header.stamp = now;
  hanp_trajectory_array.header.frame_id = frame_id;

  for (auto &human_plan_traj_combined : humans_plans_traj_combined) {
    hanp_msgs::HumanTrajectory hanp_trajectory;
    hanp_trajectory.header.stamp = now;
    hanp_trajectory.header.frame_id = frame_id;
    hanp_trajectory.trajectory.header.stamp = now;
    hanp_trajectory.trajectory.header.frame_id = frame_id;

    for (auto human_pose : human_plan_traj_combined.plan_before) {
      hanp_msgs::TrajectoryPoint hanp_trajectory_point;
      hanp_trajectory_point.transform.translation.x =
          human_pose.pose.position.x;
      hanp_trajectory_point.transform.translation.y =
          human_pose.pose.position.y;
      hanp_trajectory_point.transform.translation.z =
          human_pose.pose.position.z;
      hanp_trajectory_point.transform.rotation = human_pose.pose.orientation;
      hanp_trajectory_point.time_from_start.fromSec(-1.0);
      hanp_trajectory.trajectory.points.push_back(hanp_trajectory_point);
    }

    for (auto human_traj_point :
         human_plan_traj_combined.optimized_trajectory) {
      hanp_msgs::TrajectoryPoint hanp_trajectory_point;
      hanp_trajectory_point.transform.translation.x =
          human_traj_point.pose.position.x;
      hanp_trajectory_point.transform.translation.y =
          human_traj_point.pose.position.y;
      hanp_trajectory_point.transform.translation.z =
          human_traj_point.pose.position.z;
      hanp_trajectory_point.transform.rotation =
          human_traj_point.pose.orientation;
      hanp_trajectory_point.velocity = human_traj_point.velocity;
      hanp_trajectory_point.time_from_start = human_traj_point.time_from_start;
      hanp_trajectory.trajectory.points.push_back(hanp_trajectory_point);
    }

    for (auto human_pose : human_plan_traj_combined.plan_after) {
      hanp_msgs::TrajectoryPoint hanp_trajectory_point;
      hanp_trajectory_point.transform.translation.x =
          human_pose.pose.position.x;
      hanp_trajectory_point.transform.translation.y =
          human_pose.pose.position.y;
      hanp_trajectory_point.transform.translation.z =
          human_pose.pose.position.z;
      hanp_trajectory_point.transform.rotation = human_pose.pose.orientation;
      hanp_trajectory_point.time_from_start.fromSec(-1.0);
      hanp_trajectory.trajectory.points.push_back(hanp_trajectory_point);
    }

    if (!hanp_trajectory.trajectory.points.empty()) {
      hanp_trajectory.id = human_plan_traj_combined.id;
      hanp_trajectory_array.trajectories.push_back(hanp_trajectory);
    }
  }

  if (!hanp_trajectory_array.trajectories.empty()) {
    humans_local_plans_pub_.publish(hanp_trajectory_array);
  }
}

void TebVisualization::publishRobotFootprintModel(
    const PoseSE2 &current_pose, const BaseRobotFootprintModel &robot_model,
    const std::string &ns) {
  if (printErrorWhenNotInitialized())
    return;

  std::vector<visualization_msgs::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers);
  if (markers.empty())
    return;

  int idx = 0;
  for (std::vector<visualization_msgs::Marker>::iterator
           marker_it = markers.begin();
       marker_it != markers.end(); ++marker_it, ++idx) {
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = ros::Time::now();
    marker_it->action = visualization_msgs::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = ros::Duration(2.0);
    teb_marker_pub_.publish(*marker_it);
  }
}

void TebVisualization::publishObstacles(const ObstContainer &obstacles) const {
  if (obstacles.empty() || printErrorWhenNotInitialized())
    return;

  // Visualize point obstacles
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);

    for (ObstContainer::const_iterator obst = obstacles.begin();
         obst != obstacles.end(); ++obst) {
      boost::shared_ptr<PointObstacle> pobst =
          boost::dynamic_pointer_cast<PointObstacle>(*obst);
      if (!pobst)
        continue;
      geometry_msgs::Point point;
      point.x = pobst->x();
      point.y = pobst->y();
      point.z = 0;
      marker.points.push_back(point);
    }

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    teb_marker_pub_.publish(marker);
  }

  // Visualize line obstacles
  {
    unsigned int idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin();
         obst != obstacles.end(); ++obst) {
      boost::shared_ptr<LineObstacle> pobst =
          boost::dynamic_pointer_cast<LineObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "LineObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      geometry_msgs::Point start;
      start.x = pobst->start().x();
      start.y = pobst->start().y();
      start.z = 0;
      marker.points.push_back(start);
      geometry_msgs::Point end;
      end.x = pobst->end().x();
      end.y = pobst->end().y();
      end.z = 0;
      marker.points.push_back(end);

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      teb_marker_pub_.publish(marker);
    }
  }

  // Visualize polygon obstacles
  {
    unsigned int idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin();
         obst != obstacles.end(); ++obst) {
      boost::shared_ptr<PolygonObstacle> pobst =
          boost::dynamic_pointer_cast<PolygonObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "PolyObstacles";
      marker.id = idx++;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);

      for (Point2dContainer::const_iterator vertex = pobst->vertices().begin();
           vertex != pobst->vertices().end(); ++vertex) {
        geometry_msgs::Point point;
        point.x = vertex->x();
        point.y = vertex->y();
        point.z = 0;
        marker.points.push_back(point);
      }

      // Also add last point to close the polygon
      // but only if polygon has more than 2 points (it is not a line)
      if (pobst->vertices().size() > 2) {
        geometry_msgs::Point point;
        point.x = pobst->vertices().front().x();
        point.y = pobst->vertices().front().y();
        point.z = 0;
        marker.points.push_back(point);
      }
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;

      teb_marker_pub_.publish(marker);
    }
  }
}

void TebVisualization::publishViaPoints(
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>> &via_points,
    const std::string &ns) const {
  if (via_points.empty() || printErrorWhenNotInitialized())
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);

  for (std::size_t i = 0; i < via_points.size(); ++i) {
    geometry_msgs::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  teb_marker_pub_.publish(marker);
}

void TebVisualization::publishTebContainer(
    const TebOptPlannerContainer &teb_planner, const std::string &ns) {
  if (printErrorWhenNotInitialized())
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  // Iterate through teb pose sequence
  for (TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin();
       it_teb != teb_planner.end(); ++it_teb) {
    // iterate single poses
    PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
    PoseSequence::const_iterator it_pose_end =
        it_teb->get()->teb().poses().end();
    std::advance(it_pose_end, -1); // since we are interested in line segments,
                                   // reduce end iterator by one.
    while (it_pose != it_pose_end) {
      geometry_msgs::Point point_start;
      point_start.x = (*it_pose)->x();
      point_start.y = (*it_pose)->y();
      point_start.z = 0;
      marker.points.push_back(point_start);

      geometry_msgs::Point point_end;
      point_end.x = (*boost::next(it_pose))->x();
      point_end.y = (*boost::next(it_pose))->y();
      point_end.z = 0;
      marker.points.push_back(point_end);
      ++it_pose;
    }
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_.publish(marker);
}

void TebVisualization::publishFeedbackMessage(
    const std::vector<boost::shared_ptr<TebOptimalPlanner>> &teb_planners,
    unsigned int selected_trajectory_idx, const ObstContainer &obstacles) {
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = selected_trajectory_idx;

  msg.trajectories.resize(teb_planners.size());

  // Iterate through teb pose sequence
  std::size_t idx_traj = 0;
  for (TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin();
       it_teb != teb_planners.end(); ++it_teb, ++idx_traj) {
    msg.trajectories[idx_traj].header = msg.header;
    it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  }

  // add obstacles
  msg.obstacles.resize(obstacles.size());
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles[i].polygon);
  }

  feedback_pub_.publish(msg);
}

void TebVisualization::publishFeedbackMessage(
    const TebOptimalPlanner &teb_planner, const ObstContainer &obstacles) {
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = 0;

  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);

  // add obstacles
  msg.obstacles.resize(obstacles.size());
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles[i].polygon);
  }

  feedback_pub_.publish(msg);
}

inline bool TebVisualization::printErrorWhenNotInitialized() const {
  if (!initialized_) {
    ROS_ERROR("TebVisualization class not initialized. You must call "
              "initialize or an appropriate constructor");
    return true;
  }
  return false;
}

void TebVisualization::clearingTimerCB(const ros::TimerEvent &event) {
  if ((last_publish_robot_global_plan !=
       cfg_->visualization.publish_robot_global_plan) &&
      !cfg_->visualization.publish_robot_global_plan) {
    // clear robot global plans
    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = cfg_->map_frame;
    global_plan_pub_.publish(empty_path);
  }
  last_publish_robot_global_plan =
      cfg_->visualization.publish_robot_global_plan;

  if ((last_publish_robot_local_plan !=
       cfg_->visualization.publish_robot_local_plan) &&
      !cfg_->visualization.publish_robot_local_plan) {
    // clear robot local plans
    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = cfg_->map_frame;
    local_plan_pub_.publish(empty_path);
  }
  last_publish_robot_local_plan = cfg_->visualization.publish_robot_local_plan;

  if ((last_publish_robot_local_plan_poses !=
       cfg_->visualization.publish_robot_local_plan_poses) &&
      !cfg_->visualization.publish_robot_local_plan_poses) {
    // clear robot local plan poses
    geometry_msgs::PoseArray empty_pose_array;
    empty_pose_array.header.stamp = ros::Time::now();
    empty_pose_array.header.frame_id = cfg_->map_frame;
    teb_poses_pub_.publish(empty_pose_array);
  }
  last_publish_robot_local_plan_poses =
      cfg_->visualization.publish_robot_local_plan_poses;

  if ((last_publish_human_global_plans !=
       cfg_->visualization.publish_human_global_plans) &&
      !cfg_->visualization.publish_human_global_plans) {
    // clear human global plans
    hanp_msgs::HumanPathArray empty_path_array;
    empty_path_array.header.stamp = ros::Time::now();
    empty_path_array.header.frame_id = cfg_->map_frame;
    humans_global_plans_pub_.publish(empty_path_array);
  }
  last_publish_human_global_plans =
      cfg_->visualization.publish_human_global_plans;

  if ((last_publish_human_local_plans !=
       cfg_->visualization.publish_human_local_plans) &&
      !cfg_->visualization.publish_human_local_plans) {
    // clear human local plans
    hanp_msgs::HumanTrajectoryArray empty_trajectory_array;
    empty_trajectory_array.header.stamp = ros::Time::now();
    empty_trajectory_array.header.frame_id = cfg_->map_frame;
    humans_local_plans_pub_.publish(empty_trajectory_array);
  }
  last_publish_human_local_plans =
      cfg_->visualization.publish_human_local_plans;

  if ((last_publish_human_local_plan_poses !=
       cfg_->visualization.publish_human_local_plan_poses) &&
      !cfg_->visualization.publish_human_local_plan_poses) {
    // clear human local plan poses
    geometry_msgs::PoseArray empty_pose_array;
    empty_pose_array.header.stamp = ros::Time::now();
    empty_pose_array.header.frame_id = cfg_->map_frame;
    humans_tebs_poses_pub_.publish(empty_pose_array);
  }
  last_publish_human_local_plan_poses =
      cfg_->visualization.publish_human_local_plan_poses;
}

} // namespace teb_local_planner
