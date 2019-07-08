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

#define PREDICT_SERVICE_NAME "/human_pose_prediction/predict_human_poses"
#define RESET_PREDICTION_SERVICE_NAME                                          \
  "/human_pose_prediction/reset_external_paths"
#define PUBLISH_MARKERS_SRV_NAME                                               \
  "/human_pose_prediction/publish_prediction_markers"
#define OPTIMIZE_SRV_NAME "optimize"
#define APPROACH_SRV_NAME "set_approach_id"
#define OP_COSTS_TOPIC "optimization_costs"
#define ROB_POS_TOPIC "Robot_Pose"
#define DEFAULT_HUMAN_SEGMENT hanp_msgs::TrackedSegmentType::TORSO
#define THROTTLE_RATE 5.0 // seconds

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS,
                        nav_core::BaseLocalPlanner)

namespace teb_local_planner {

TebLocalPlannerROS::TebLocalPlannerROS()
    : costmap_ros_(NULL), tf2_(NULL), costmap_model_(NULL),
      costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
      dynamic_recfg_(NULL), goal_reached_(false), no_infeasible_plans_(0),
      initialized_(false) {}

TebLocalPlannerROS::~TebLocalPlannerROS() {}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig &config,
                                       uint32_t level) {
  cfg_.reconfigure(config);
}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf2,
                                    costmap_2d::Costmap2DROS *costmap_ros) {
  // check if the plugin is already initialized
  if (!initialized_) {
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);

    // get parameters of TebConfig via the nodehandle and override the default
    // config
    cfg_.loadRosParamFromNodeHandle(nh);

    // reserve some memory for obstacles
    obstacles_.reserve(500);

    // init some variables
    tf2_ = tf2;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

     costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);


     global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // create visualization instance
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_.map_frame));

    // create robot footprint/contour model for optimization
    RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh);

    CircularRobotFootprintPtr human_model = NULL;
    auto human_radius = cfg_.human.radius;
    if (human_radius < 0.0) {
      ROS_WARN("human radius is set to negative, using 0.0");
      human_radius = 0.0;
    }
    human_model = boost::make_shared<CircularRobotFootprint>(human_radius);

    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning) {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(
          cfg_, &obstacles_, robot_model, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    } else {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(
          cfg_, &obstacles_, robot_model, visualization_, &via_points_,
          human_model, &humans_via_points_map_));
      planner_->local_weight_optimaltime_ = cfg_.optim.weight_optimaltime;
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }

    // // init other variables
    // tf2_ = tf2;
    // costmap_ros_ = costmap_ros;
    // costmap_ =
    //     costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    //
    // costmap_model_ =
    //     boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
    //
    // global_frame_ = costmap_ros_->getGlobalFrameID();
    // cfg_.map_frame = global_frame_; // TODO
    // robot_base_frame_ = costmap_ros_->getBaseFrameID();

    // Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty()) {
      try {
        costmap_converter_ = costmap_converter_loader_.createInstance(
            cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(
            cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle
        // namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->initialize(
            ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);

        costmap_converter_->startWorker(
            ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_,
            cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin "
                        << cfg_.obstacles.costmap_converter_plugin
                        << " loaded.");
      } catch (pluginlib::PluginlibException &ex) {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All "
                 "occupied costmap cells are treaten as point obstacles. Error "
                 "message: %s",
                 ex.what());
        costmap_converter_.reset();
      }
    } else
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap "
               "cells are treaten as point obstacles.");

    // Get footprint of the robot and minimum and maximum distance from the
    // center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(
        footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared<
        dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType
        cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);

    // validate optimization footprint and costmap footprint
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe(
        "obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // setup human prediction client with persistent connection
    predict_humans_client_ =
        nh.serviceClient<hanp_prediction::HumanPosePredict>(
            PREDICT_SERVICE_NAME, true);
    reset_humans_prediction_client_ =
        nh.serviceClient<std_srvs::Empty>(RESET_PREDICTION_SERVICE_NAME, true);
    publish_predicted_markers_client_ =
        nh.serviceClient<std_srvs::SetBool>(PUBLISH_MARKERS_SRV_NAME, true);

    optimize_server_ = nh.advertiseService(
        OPTIMIZE_SRV_NAME, &TebLocalPlannerROS::optimizeStandalone, this);
    approach_server_ = nh.advertiseService(
        APPROACH_SRV_NAME, &TebLocalPlannerROS::setApproachID, this);

    op_costs_pub_ = nh.advertise<teb_local_planner::OptimizationCostArray>(
        OP_COSTS_TOPIC, 1);

    robot_pose_pub_ = nh.advertise<geometry_msgs::Pose>(
        ROB_POS_TOPIC, 1);

    last_call_time_ =
        ros::Time::now() - ros::Duration(cfg_.human.pose_prediction_reset_time);

    last_omega_sign_change_ =
        ros::Time::now() -
        ros::Duration(cfg_.optim.omega_chage_time_seperation);
    last_omega_ = 0.0;

    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  } else {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}

bool TebLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  // check if plugin is initialized
  if (!initialized_) {
    ROS_ERROR("teb_local_planner has not been initialized, please call "
              "initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently
  // whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the
  // trajectory or not within each velocity computation step.

  // reset goal_reached_ flag
  goal_reached_ = false;

  return true;
}

bool TebLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist &cmd_vel) {
  auto start_time = ros::Time::now();
  if ((start_time - last_call_time_).toSec() >
      cfg_.human.pose_prediction_reset_time) {
    resetHumansPrediction();
  }
  last_call_time_ = start_time;

  // check if plugin initialized
  if (!initialized_) {
    ROS_ERROR("teb_local_planner has not been initialized, please call "
              "initialize() before using this planner");
    return false;
  }

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  goal_reached_ = false;

  // Get robot pose
  auto pose_get_start_time = ros::Time::now();
  // tf2::Stamped<tf2::Transform> robot_pose;
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
  geometry_msgs::Pose robot_pos_msg;
  robot_pose_.toPoseMsg(robot_pos_msg);
  robot_pose_pub_.publish(robot_pos_msg);
  auto pose_get_time = ros::Time::now() - pose_get_start_time;

  // Get robot velocity
  auto vel_get_start_time = ros::Time::now();
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  tf::Pose robot_vel_tf_pose;
  tf::poseMsgToTF(robot_vel_tf.pose,robot_vel_tf_pose);
  robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
  robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
  robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
  auto vel_get_time = ros::Time::now() - vel_get_start_time;

  // prune global plan to cut off parts of the past (spatially before the robot)
  auto prune_start_time = ros::Time::now();
  tf2::Stamped<tf2::Transform> tf_robot_pose;
  tf2::fromMsg(robot_pose,tf_robot_pose);
  pruneGlobalPlan(*tf2_, tf_robot_pose, global_plan_);
  auto prune_time = ros::Time::now() - prune_start_time;

  // Transform global plan to the frame of interest (w.r.t to the local costmap)
  auto transform_start_time = ros::Time::now();
  PlanCombined transformed_plan_combined;
  int goal_idx;
  tf2::Stamped<tf2::Transform> tf_plan_to_global;
  if (!transformGlobalPlan(*tf2_, global_plan_, tf_robot_pose, *costmap_,
                           global_frame_,
                           cfg_.trajectory.max_global_plan_lookahead_dist,
                           transformed_plan_combined, &goal_idx, &tf_plan_to_global)) {
    ROS_WARN(
        "Could not transform the global plan to the frame of the controller");
    return false;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
  auto transform_time = ros::Time::now() - transform_start_time;

  // Check if the horizon should be reduced this run
  auto hr1_start_time = ros::Time::now();
  // if (horizon_reduced_) {
  //   // reduce to 50 percent:
  //   // int horizon_reduction = goal_idx/2;
  //   int horizon_reduction =
  //       (int)(goal_idx * cfg_.trajectory.horizon_reduction_amount);
  //   // we have a small overhead here, since we already transformed 50% more of
  //   // the trajectory.
  //   // But that's ok for now, since we do not need to make transformGlobalPlan
  //   // more complex
  //   // and a reduced horizon should occur just rarely.
  //   int new_goal_idx_transformed_plan =
  //       int(transformed_plan.size()) - horizon_reduction - 1;
  //   goal_idx -= horizon_reduction;
  //   if (new_goal_idx_transformed_plan > 0 && goal_idx >= 0)
  //     transformed_plan.erase(transformed_plan.begin() +
  //                                new_goal_idx_transformed_plan,
  //                            transformed_plan.end());
  //   else
  //     goal_idx +=
  //         horizon_reduction; // this should not happy, but safety first ;-)
  // }
  auto hr1_time = ros::Time::now() - hr1_start_time;

  auto other_start_time = ros::Time::now();
  // check if global goal is reached
  tf2::Stamped<tf2::Transform> global_goal;
  tf2::fromMsg(global_plan_.back(), global_goal);
  global_goal.setData(tf_plan_to_global * global_goal);
  double dx = global_goal.getOrigin().getX() - robot_pose_.x();
  double dy = global_goal.getOrigin().getY() - robot_pose_.y();
  double delta_orient = g2o::normalize_theta(
      tf2::impl::getYaw(global_goal.getRotation()) - robot_pose_.theta());
  if (fabs(std::sqrt(dx * dx + dy * dy)) <
          cfg_.goal_tolerance.xy_goal_tolerance &&
      fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance) {
    goal_reached_ = true;
    return true;
  }


  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);

  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    return false;
  }

  // Get current goal point (last point of the transformed plan)
  tf2::Stamped<tf2::Transform> goal_point;
  tf2::fromMsg(transformed_plan.back(), goal_point);
  robot_goal_.x() = goal_point.getOrigin().getX();
  robot_goal_.y() = goal_point.getOrigin().getY();
  if (cfg_.trajectory.global_plan_overwrite_orientation) {
    robot_goal_.theta() = estimateLocalGoalOrientation(
        global_plan_, goal_point, goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual
    // goal (enable using the plan as initialization)
    transformed_plan.back().pose.orientation =
        tf::createQuaternionMsgFromYaw(robot_goal_.theta());
    }
  else
  {
    robot_goal_.theta() = tf2::impl::getYaw(goal_point.getRotation());
  }

  // clear currently existing obstacles
  obstacles_.clear();
  auto other_time = ros::Time::now() - other_start_time;

  // Update obstacle container with costmap information or polygons provided by
  // a costmap_converter plugin
  auto cc_start_time = ros::Time::now();
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();

  // also consider custom obstacles (must be called after other updates, since
  // the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  auto cc_time = ros::Time::now() - cc_start_time;

  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // update humans
  auto human_start_time = ros::Time::now();
  std::vector<HumanPlanCombined> transformed_human_plans;
  HumanPlanVelMap transformed_human_plan_vel_map;
  switch (cfg_.planning_mode) {
  case 0:
    break;
  case 1: {
    hanp_prediction::HumanPosePredict predict_srv;
    if (cfg_.human.use_external_prediction) {
      if (cfg_.human.predict_human_behind_robot) {
        predict_srv.request.type =
            hanp_prediction::HumanPosePredictRequest::BEHIND_ROBOT;
      } else {
        predict_srv.request.type =
            hanp_prediction::HumanPosePredictRequest::EXTERNAL;
      }
    } else {
      double traj_size = 10,
             predict_time = 5.0; // TODO: make these values configurable
      for (double i = 1.0; i <= traj_size; ++i) {
        predict_srv.request.predict_times.push_back(predict_time *
                                                    (i / traj_size));
      }
      predict_srv.request.type =
          hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;
    }

    std_srvs::SetBool publish_predicted_markers_srv;
    publish_predicted_markers_srv.request.data =
        publish_predicted_human_markers_;
    if (!publish_predicted_markers_client_ &&
        publish_predicted_markers_client_.call(publish_predicted_markers_srv)) {
      ROS_WARN("Failed to call %s service, is human prediction server running?",
               PUBLISH_MARKERS_SRV_NAME);
    }

    if (predict_humans_client_ && predict_humans_client_.call(predict_srv)) {
      tf2::Stamped<tf2::Transform> tf_human_plan_to_global;
      for (auto predicted_humans_poses :
           predict_srv.response.predicted_humans_poses) {
        // transform human plans
        HumanPlanCombined human_plan_combined;
        auto &transformed_vel = predicted_humans_poses.start_velocity;
        if (!transformHumanPlan(*tf2_, robot_pose, *costmap_, global_frame_,
                                predicted_humans_poses.poses,
                                human_plan_combined, transformed_vel,
                                &tf_human_plan_to_global)) {
          ROS_WARN("Could not transform the human %ld plan to the frame of the "
                   "controller",
                   predicted_humans_poses.id);
          continue;
        }

        human_plan_combined.id = predicted_humans_poses.id;
        transformed_human_plans.push_back(human_plan_combined);

        PlanStartVelGoalVel plan_start_vel_goal_vel;
        plan_start_vel_goal_vel.plan = human_plan_combined.plan_to_optimize;
        plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
        if (human_plan_combined.plan_after.size() > 0) {
          plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
        }
        transformed_human_plan_vel_map[human_plan_combined.id] =
            plan_start_vel_goal_vel;
      }
    } else {
      ROS_WARN_THROTTLE(
          THROTTLE_RATE,
          "Failed to call %s service, is human prediction server running?",
          PREDICT_SERVICE_NAME);

      // re-initialize the service
      // predict_humans_client_ =
      // nh.serviceClient<hanp_prediction::HumanPosePredict>(PREDICT_SERVICE_NAME,
      // true);
    }
    updateHumanViaPointsContainers(transformed_human_plan_vel_map,
                                   cfg_.trajectory.global_plan_viapoint_sep);
    break;
  }
  case 2: {
    hanp_prediction::HumanPosePredict predict_srv;
    predict_srv.request.predict_times.push_back(0.0);
    predict_srv.request.predict_times.push_back(5.0);
    predict_srv.request.type =
        hanp_prediction::HumanPosePredictRequest::VELOCITY_OBSTACLE;

    // setup marker publishing
    std_srvs::SetBool publish_predicted_markers_srv;
    publish_predicted_markers_srv.request.data =
        publish_predicted_human_markers_;
    if (!publish_predicted_markers_client_ &&
        publish_predicted_markers_client_.call(publish_predicted_markers_srv)) {
      ROS_WARN("Failed to call %s service, is human prediction server running?",
               PUBLISH_MARKERS_SRV_NAME);
    }

    // call human prediction server
    if (predict_humans_client_ && predict_humans_client_.call(predict_srv)) {
      for (auto predicted_humans_poses :
           predict_srv.response.predicted_humans_poses) {
        if (predicted_humans_poses.id == cfg_.approach.approach_id) {
          geometry_msgs::PoseStamped transformed_human_pose;
          if (!transformHumanPose(*tf2_, global_frame_,
                                  predicted_humans_poses.poses.front(),
                                  transformed_human_pose)) {
            ROS_WARN(
                "Could not transform the human %ld pose to controller frame",
                predicted_humans_poses.id);
          }

          PlanStartVelGoalVel plan_start_vel_goal_vel;
          plan_start_vel_goal_vel.plan.push_back(transformed_human_pose);
          transformed_human_plan_vel_map[predicted_humans_poses.id] =
              plan_start_vel_goal_vel;

          // update global plan of the robot
          // find position in front of the human
          tf2::Transform tf_human_pose, tf_approach_pose[3];
            geometry_msgs::PoseStamped approach_pose[3];
            for (int i=0; i < 3; i++){
                tf2::fromMsg(transformed_human_pose.pose, tf_human_pose);
                tf_approach_pose[i].setOrigin(
                        tf2::Vector3(cfg_.approach.approach_dist + (2 - i) * 0.3, 0.0, 0.0));
		tf2::Quaternion approachQuaternion;
                approachQuaternion.setEuler(cfg_.approach.approach_angle,0.0,0.0);
                tf_approach_pose[i].setRotation(approachQuaternion);
                tf_approach_pose[i] = tf_human_pose * tf_approach_pose[i];
                tf2::toMsg(tf_approach_pose[i], approach_pose[i].pose);
                approach_pose[i].header = transformed_human_pose.header;
            }



          // ROS_INFO("human pose: x=%.2f, y=%.2f, theta=%.2f, frame=%s",
          //          transformed_human_pose.pose.position.x,
          //          transformed_human_pose.pose.position.y,
          //          tf::getYaw(transformed_human_pose.pose.orientation),
          //          transformed_human_pose.header.frame_id.c_str());
          // ROS_INFO("approach pose: x=%.2f, y=%.2f, theta=%.2f, frame=%s",
          //          approach_pose.pose.position.x,
          //          approach_pose.pose.position.y,
          //          tf::getYaw(approach_pose.pose.orientation),
          //          approach_pose.header.frame_id.c_str());

          // add approach pose to the robot plan, only within reachable distance
          // auto &plan_goal = transformed_plan.back().pose;
          // auto &approach_goal = approach_pose[2].pose;
          tf2::Transform plan_goal,approach_goal;
          tf2::fromMsg(transformed_plan.back().pose,plan_goal);
          tf2::fromMsg(approach_pose[2].pose,approach_goal);
          double lin_dist = std::abs(
              std::hypot(plan_goal.getOrigin().getX() - approach_goal.getOrigin().getX(),
                         plan_goal.getOrigin().getY() - approach_goal.getOrigin().getY()));
          double ang_dist = std::abs(angles::shortest_angular_distance(
              tf2::impl::getYaw(plan_goal.getRotation()),
              tf2::impl::getYaw(approach_goal.getRotation())));
          // ROS_INFO("lin_dist=%.2f, ang_dist=%.2f", lin_dist, ang_dist);
            tf2::Transform tf_approach_global[3];
            geometry_msgs::PoseStamped approach_pose_global[3];
          if (lin_dist > cfg_.approach.approach_dist_tolerance ||
              ang_dist > cfg_.approach.approach_angle_tolerance) {
              for (int i = 0; i < 3; i++) {
                  transformed_plan.push_back(approach_pose[i]);

                  // get approach poses in to the frame of global pla
                  tf_approach_global[i] = tf_plan_to_global.inverse() * tf_approach_pose[i];

                  tf2::toMsg(tf_approach_global[i], approach_pose_global[i].pose);
                  approach_pose_global[i].header = global_plan_.back().header;
              }

            // prune and update global plan
            auto global_plan_it = global_plan_.begin();
            double last_dist = std::numeric_limits<double>::infinity();
            while (global_plan_it != global_plan_.end()) {
              auto &p_pos = (*global_plan_it).pose.position;
              auto &a_pos = approach_pose_global[0].pose.position;
              double pa_dist = std::hypot(p_pos.x - a_pos.x, p_pos.y - a_pos.y);
              if (pa_dist > last_dist) {
                break;
              }
              last_dist = pa_dist;
              global_plan_it++;
            }
            global_plan_.erase(global_plan_it, global_plan_.end());
              for (int i = 0; i < 3; i++) {
                  global_plan_.push_back(approach_pose_global[i]);
              }
            ROS_INFO("Global plan modified for approach behavior");
          }
        }
      }
    } else {
      ROS_WARN_THROTTLE(
          THROTTLE_RATE,
          "Failed to call %s service, is human prediction server running?",
          PREDICT_SERVICE_NAME);
    }
    // TODO: check if plan-map is not empty
    break;
  }
  default:
    break;
  }
  auto human_time = ros::Time::now() - human_start_time;

  // update via-points container
  auto via_start_time = ros::Time::now();
  // overwrite/update start of the transformed plan with the actual robot
  // position (allows using the plan as initial trajectory)
  // tf::poseTFToMsg(robot_pose, transformed_plan.front().pose);
  transformed_plan.front()=robot_pose;
  updateViaPointsContainer(transformed_plan,
                           cfg_.trajectory.global_plan_viapoint_sep);
  auto via_time = ros::Time::now() - via_start_time;

  // Now perform the actual planning
  auto plan_start_time = ros::Time::now();
  // bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_,
  // cfg_.goal_tolerance.free_goal_vel); // straight line init
  teb_local_planner::OptimizationCostArray op_costs;
  bool success = planner_->plan(transformed_plan, &robot_vel_,
                                cfg_.goal_tolerance.free_goal_vel,
                                &transformed_human_plan_vel_map, &op_costs);
  if (!success) {
    planner_->clearPlanner(); // force reinitialization for next time
    ROS_WARN("teb_local_planner was not able to obtain a local plan for the "
             "current setting.");
    return false;
  }
  op_costs_pub_.publish(op_costs);
  auto plan_time = ros::Time::now() - plan_start_time;

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  if (cfg_.planning_mode == 1) {
    visualization_->publishHumanGlobalPlans(transformed_human_plans);
    std::vector<HumanPlanTrajCombined> human_plans_traj_array;
    for (auto &human_plan_combined : transformed_human_plans) {
      HumanPlanTrajCombined human_plan_traj_combined;
      human_plan_traj_combined.id = human_plan_combined.id;
      human_plan_traj_combined.plan_before = human_plan_combined.plan_before;
      planner_->getFullHumanTrajectory(
          human_plan_traj_combined.id,
          human_plan_traj_combined.optimized_trajectory);
      human_plan_traj_combined.plan_after = human_plan_combined.plan_after;
      human_plans_traj_array.push_back(human_plan_traj_combined);
    }
    visualization_->publishHumanTrajectories(human_plans_traj_array);
  }
  auto viz_time = ros::Time::now() - viz_start_time;

  // Undo temporary horizon reduction
  auto hr2_start_time = ros::Time::now();
  // if (horizon_reduced_ &&
  //     (ros::Time::now() - horizon_reduced_stamp_).toSec() >= 5 &&
  //     !planner_->isHorizonReductionAppropriate(
  //         transformed_plan)) // 10s are hardcoded for now...
  // {
  //   horizon_reduced_ = false;
  //   planner_->local_weight_optimaltime_ = cfg_.optim.weight_optimaltime;
  //   ROS_INFO("Switching back to full horizon length.");
  // }
  ++no_infeasible_plans_;
  time_last_infeasible_plan_ = ros::Time::now();

  return false;

  auto hr2_time = ros::Time::now() - hr2_start_time;

  // Check feasibility (but within the first few states only)
  auto fsb_start_time = ros::Time::now();
  bool feasible = planner_->isTrajectoryFeasible(
      costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
      robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible) {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    if (!horizon_reduced_ && cfg_.trajectory.shrink_horizon_backup &&
        planner_->isHorizonReductionAppropriate(transformed_plan)) {
      horizon_reduced_ = true;
      planner_->local_weight_optimaltime_ = 0.4;

      ROS_WARN("TebLocalPlannerROS: trajectory is not feasible, using slower "
               "trajectory for next 5s...");
      horizon_reduced_stamp_ = ros::Time::now();
      return true; // commanded velocity is zero for this step
    }
    // now we reset everything to start again with the initialization of new
    // trajectories.
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting "
             "planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();

    return false;
  }
  auto fsb_time = ros::Time::now() - fsb_start_time;

  // Get the velocity command for this sampling interval
  auto vel_start_time = ros::Time::now();
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)) {
    planner_->clearPlanner();
    ROS_WARN(
        "TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    return false;
  }

  // Saturate velocity, if the optimization results violates the constraints
  // (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
                   cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a
  // soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The
  // user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel) {
    cmd_vel.angular.z = convertTransRotVelToSteeringAngle(
        cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.wheelbase,
        0.95 * cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.angular.z)) {
      cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. "
               "Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();

      return false;
    }
  }
  auto vel_time = ros::Time::now() - vel_start_time;

  auto total_time = ros::Time::now() - start_time;
  ROS_DEBUG_STREAM_COND(
      total_time.toSec() > 0.1,
      "\tcompute velocity times:\n"
          << "\t\ttotal time                   "
          << std::to_string(total_time.toSec()) << "\n"
          << "\t\tpose get time                "
          << std::to_string(pose_get_time.toSec()) << "\n"
          << "\t\tvel get time                 "
          << std::to_string(vel_get_time.toSec()) << "\n"
          << "\t\tprune time                   "
          << std::to_string(prune_time.toSec()) << "\n"
          << "\t\ttransform time               "
          << std::to_string(transform_time.toSec()) << "\n"
          << "\t\thorizon setup time           "
          << std::to_string((hr1_time + hr2_time).toSec()) << "\n"
          << "\t\tother time                   "
          << std::to_string(other_time.toSec()) << "\n"
          << "\t\tcostmap convert time         "
          << std::to_string(cc_time.toSec()) << "\n"
          << "\t\tvia points time              "
          << std::to_string(via_time.toSec()) << "\n"
          << "\t\thuman time                   "
          << std::to_string(human_time.toSec()) << "\n"
          << "\t\tplanning time                "
          << std::to_string(plan_time.toSec()) << "\n"
          << "\t\tplan feasibility check time  "
          << std::to_string(fsb_time.toSec()) << "\n"
          << "\t\tvelocity extract time        "
          << std::to_string(vel_time.toSec()) << "\n"
          << "\t\tvisualization publish time   "
          << std::to_string(viz_time.toSec()) << "\n=========================");

  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;

   // Now visualize everything

  auto viz_start_time = ros::Time::now();
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  return true;
}

bool TebLocalPlannerROS::isGoalReached() {
  if (goal_reached_) {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    resetHumansPrediction();
    return true;
  }
  return false;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap() {
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles) {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

    for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i) {
      for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j) {
        if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs - robot_pose_.position();
          if (obs_dir.dot(robot_orient) < 0 &&
              obs_dir.norm() >
                  cfg_.obstacles.costmap_obstacles_behind_robot_dist)
            continue;

          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter() {
  if (!costmap_converter_)
    return;

  // Get obstacles from costmap converter
  costmap_converter::PolygonContainerConstPtr polygons =
      costmap_converter_->getPolygons();
  if (!polygons)
    return;

  for (std::size_t i = 0; i < polygons->size(); ++i) {
    if (polygons->at(i).points.size() == 1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(
          polygons->at(i).points[0].x, polygons->at(i).points[0].y)));
    } else if (polygons->at(i).points.size() == 2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(
          polygons->at(i).points[0].x, polygons->at(i).points[0].y,
          polygons->at(i).points[1].x, polygons->at(i).points[1].y)));
    } else if (polygons->at(i).points.size() > 2) // Real polygon
    {
      PolygonObstacle *polyobst = new PolygonObstacle;
      for (std::size_t j = 0; j < polygons->at(i).points.size(); ++j) {
        polyobst->pushBackVertex(polygons->at(i).points[j].x,
                                 polygons->at(i).points[j].y);
      }
      polyobst->finalizePolygon();
      obstacles_.push_back(ObstaclePtr(polyobst));
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles() {
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty()) {
    // We only use the global header to specify the obstacle coordinate system
    // instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try {
        geometry_msgs::TransformStamped obstacle_to_map;
      // tf2_->waitForTransform(global_frame_, ros::Time(0),
      //                       custom_obstacle_msg_.header.frame_id, ros::Time(0),
      //                       custom_obstacle_msg_.header.frame_id,
      //                       ros::Duration(0.5));

       obstacle_to_map = tf2_->lookupTransform(global_frame_, ros::Time(0), custom_obstacle_msg_.header.frame_id,
		       ros::Time(0), custom_obstacle_msg_.header.frame_id,ros::Duration(0.5));
        tf2::Stamped< tf2::Transform > obstacle_to_map_;

       tf2::fromMsg(obstacle_to_map,obstacle_to_map_);
       geometry_msgs::PoseStamped obstacle_to_map_pose;
       obstacle_to_map_pose = tf2::toMsg(obstacle_to_map_,obstacle_to_map_pose);
       tf2::fromMsg(obstacle_to_map_pose.pose,obstacle_to_map_eig);

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      obstacle_to_map_eig.setIdentity();
    }

    for (std::vector<geometry_msgs::PolygonStamped>::const_iterator obst_it =
             custom_obstacle_msg_.obstacles.begin();
         obst_it != custom_obstacle_msg_.obstacles.end(); ++obst_it) {
      if (obst_it->polygon.points.size() == 1) // point
      {
        Eigen::Vector3d pos(obst_it->polygon.points.front().x,
                            obst_it->polygon.points.front().y,
                            obst_it->polygon.points.front().z);
        obstacles_.push_back(ObstaclePtr(
            new PointObstacle((obstacle_to_map_eig * pos).head(2))));
      } else if (obst_it->polygon.points.size() == 2) // line
      {
        Eigen::Vector3d line_start(obst_it->polygon.points.front().x,
                                   obst_it->polygon.points.front().y,
                                   obst_it->polygon.points.front().z);
        Eigen::Vector3d line_end(obst_it->polygon.points.back().x,
                                 obst_it->polygon.points.back().y,
                                 obst_it->polygon.points.back().z);
        obstacles_.push_back(ObstaclePtr(
            new LineObstacle((obstacle_to_map_eig * line_start).head(2),
                             (obstacle_to_map_eig * line_end).head(2))));
      } else // polygon
      {
        PolygonObstacle *polyobst = new PolygonObstacle;
        for (int i = 0; i < (int)obst_it->polygon.points.size(); ++i) {
          Eigen::Vector3d pos(obst_it->polygon.points[i].x,
                              obst_it->polygon.points[i].y,
                              obst_it->polygon.points[i].z);
          polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(
    const std::vector<geometry_msgs::PoseStamped> &transformed_plan,
    double min_separation) {
  via_points_.clear();

  if (min_separation < 0)
    return;

  std::size_t prev_idx = 0;
  for (std::size_t i = 1; i < transformed_plan.size();
       ++i) // skip first one, since we do not need any point before the first
            // min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d(transformed_plan[prev_idx].pose.position,
                          transformed_plan[i].pose.position) < min_separation)
      continue;

    // add via-point
    via_points_.push_back(Eigen::Vector2d(transformed_plan[i].pose.position.x,
                                          transformed_plan[i].pose.position.y));
    prev_idx = i;
  }
}

void TebLocalPlannerROS::updateHumanViaPointsContainers(
    const HumanPlanVelMap &transformed_human_plan_vel_map,
    double min_separation) {
  if (min_separation < 0)
    return;

  // reset via-points for known humans, create via-points for new humans
  for (auto &transformed_human_plan_vel_kv : transformed_human_plan_vel_map) {
    auto &human_id = transformed_human_plan_vel_kv.first;
    if (humans_via_points_map_.find(human_id) != humans_via_points_map_.end())
      humans_via_points_map_[human_id].clear();
    else
      humans_via_points_map_[human_id] = ViaPointContainer();
  }

  // remove human via-points for vanished humans
  auto itr = humans_via_points_map_.begin();
  while (itr != humans_via_points_map_.end()) {
    if (transformed_human_plan_vel_map.find(itr->first) !=
        transformed_human_plan_vel_map.end())
      itr = humans_via_points_map_.erase(itr);
    else
      ++itr;
  }

  std::size_t prev_idx;
  for (auto &transformed_human_plan_vel_kv : transformed_human_plan_vel_map) {
    prev_idx = 0;
    auto &human_id = transformed_human_plan_vel_kv.first;
    auto &transformed_human_plan = transformed_human_plan_vel_kv.second.plan;
    for (std::size_t i = 1; i < transformed_human_plan.size(); ++i) {
      if (distance_points2d(transformed_human_plan[prev_idx].pose.position,
                            transformed_human_plan[i].pose.position) <
          min_separation)
        continue;

      humans_via_points_map_[human_id].push_back(
          Eigen::Vector2d(transformed_human_plan[i].pose.position.x,
                          transformed_human_plan[i].pose.position.y));
      prev_idx = i;
    }
  }
}

Eigen::Vector2d
TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose &tf_vel) {
  Eigen::Vector2d vel;
  vel.coeffRef(0) =
      std::sqrt(tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() +
                tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY());
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}

bool TebLocalPlannerROS::pruneGlobalPlan(
    const tf2_ros::Buffer &tf2, const tf2::Stamped<tf2::Transform> &global_pose,
    std::vector<geometry_msgs::PoseStamped> &global_plan,
    double dist_behind_robot) {
  if (global_plan.empty())
    return true;

  try {
    // transform robot pose into the plan frame (we do not wait here, since
    // pruning not crucial, if missed a few times)
    //geometry_msgs::PoseStamped

    geometry_msgs::TransformStamped global_to_plan_transform;
    global_to_plan_transform = tf2.lookupTransform(global_plan.front().header.frame_id,
                                                  global_pose.frame_id_, ros::Time(0),ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > global_to_plan_transform_;
    tf2::fromMsg(global_to_plan_transform,global_to_plan_transform_);

    tf2::Stamped<tf2::Transform> robot;
    robot.setData(global_to_plan_transform_ * global_pose);

    double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end()) {
      double dx = robot.getOrigin().x() - it->pose.position.x;
      double dy = robot.getOrigin().y() - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq) {
        erase_end = it;
        break;
      }
      ++it;
    }
    if (erase_end == global_plan.end()) //|| erase_end == global_plan.end() - 1)
      return false;

    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  } catch (const tf::TransformException &ex) {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n",
              ex.what());
    return false;
  }
  return true;
}

bool TebLocalPlannerROS::transformGlobalPlan(
    const tf2_ros::Buffer &tf2,
    const std::vector<geometry_msgs::PoseStamped> &global_plan,
    const tf2::Stamped<tf2::Transform> &global_pose,
    const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
    double max_plan_length,
    PlanCombined &transformed_plan_combined,
    int *current_goal_idx, tf2::Stamped<tf2::Transform> *tf_plan_to_global) const {
  // this method is a slightly modified version of
  // base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

  transformed_plan_combined.plan_to_optimize.clear();

  try {
    if (global_plan.empty()) {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform;
    // tf.waitForTransform(global_frame, ros::Time::now(),
    // plan_pose.header.frame_id, plan_pose.header.stamp,
    // plan_pose.header.frame_id, ros::Duration(0.5));
    // tf.lookupTransform(global_frame, ros::Time(),
    // plan_pose.header.frame_id, plan_pose.header.stamp,
    // plan_pose.header.frame_id, plan_to_global_transform);
    // tf.waitForTransform(global_frame, plan_pose.header.frame_id, ros::Time(0),
                        // ros::Duration(0.5));
    plan_to_global_transform = tf2.lookupTransform(global_frame, plan_pose.header.frame_id, ros::Time(0),ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > plan_to_global_transform_;
    tf2::fromMsg(plan_to_global_transform,plan_to_global_transform_);

    // let's get the pose of the robot in the frame of the plan
    tf2::Stamped<tf2::Transform> robot_pose;
    // tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);
    robot_pose.setData(plan_to_global_transform_.inverse() * global_pose);

    // we'll discard points on the plan that are outside the local costmap
    double dist_threshold =
        std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better
                            // incorporate point obstacle that are
                            // located on the border of the local costmap

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;

    tf2::Stamped<tf2::Transform> tf_pose;
    geometry_msgs::PoseStamped newer_pose;

    // we need to loop to a point on the plan that is within a certain
    // distance of the robot
    while (i < (int)global_plan.size()) {
      double x_diff =
          robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff =
          robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (new_sq_dist > sq_dist &&
          sq_dist < sq_dist_threshold) // find first distance that is greater
      {
        sq_dist = new_sq_dist;
        break;
      }
      sq_dist = new_sq_dist;

      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::fromMsg(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame;
      newer_pose = tf2::toMsg(tf_pose, newer_pose);
      transformed_plan_combined.plan_before.push_back(newer_pose);

      ++i;
    }

    // tf2::Stamped<tf2::Transform> tf_pose;
    // geometry_msgs::PoseStamped newer_pose;

    double plan_length =
        0; // check cumulative Euclidean distance along the plan

    // now we'll transform until points are outside of our distance threshold
    while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold &&
           (max_plan_length <= 0 || plan_length <= max_plan_length)) {
      const geometry_msgs::PoseStamped &pose = global_plan[i];
      tf2::fromMsg(pose, tf_pose);
      tf_pose.setData(plan_to_global_transform_ * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform_.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf2::toMsg(tf_pose, newer_pose);

      transformed_plan_combined.plan_to_optimize.push_back(newer_pose);

      double x_diff =
          robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
      double y_diff =
          robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;

      // caclulate distance to previous pose
      if (i > 0 && max_plan_length > 0)
        plan_length += distance_points2d(global_plan[i - 1].pose.position,
                                         global_plan[i].pose.position);

      ++i;
    }

    // // Modification for teb_local_planner:
    // // Return the index of the current goal point (inside the distance
    // // threshold)
    // if (current_goal_idx)
    //   *current_goal_idx =
    //       i - 1; // minus 1, since i was increased once before leaving the loop
    //
    // while (i < (int)global_plan.size()) {
    //   const geometry_msgs::PoseStamped &pose = global_plan[i];
    //   tf2::fromMsg(pose, tf_pose);
    //   tf_pose.setData(plan_to_global_transform_ * tf_pose);
    //   tf_pose.stamp_ = plan_to_global_transform_.stamp_;
    //   tf_pose.frame_id_ = global_frame;
    //   tf2::toMsg(tf_pose, newer_pose);
    //   transformed_plan_combined.plan_after.push_back(newer_pose);
    //   ++i;
    // }

    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::fromMsgF(global_plan.back(), tf_pose);
      tf_pose.setData(plan_to_global_transform * tf_pose);
      tf_pose.stamp_ = plan_to_global_transform.stamp_;
      tf_pose.frame_id_ = global_frame;
      tf2::toMsg(tf_pose, newer_pose);

      transformed_plan.push_back(newer_pose);

       // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }

     // Return the transformation from the global plan to the global planning frame if desired

    if (tf_plan_to_global)
      *tf_plan_to_global = plan_to_global_transform_;
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(), (unsigned int)global_plan.size(),
                global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}

int TebLocalPlannerROS::getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, ros::Time& time, std::string* error_string) const
{
  tf2::CompactFrameID target_id = tf2_->_lookupFrameNumber(tf::strip_leading_slash(target_frame));
  tf2::CompactFrameID source_id = tf2_->_lookupFrameNumber(tf::strip_leading_slash(source_frame));

  return tf2_->_getLatestCommonTime(source_id, target_id, time, error_string);
}

void TebLocalPlannerROS::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame,
                              const ros::Time& time, const ros::Duration& averaging_interval,
                              geometry_msgs::Twist& twist) const
{
  // ref point is origin of tracking_frame, ref_frame = obs_frame
  lookupTwist(tracking_frame, observation_frame, observation_frame, tf2::Vector3(0,0,0), tracking_frame, time, averaging_interval, twist);
};

void TebLocalPlannerROS::lookupTwist(const std::string& tracking_frame, const std::string& observation_frame, const std::string& reference_frame,
                 const tf2::Vector3 & reference_point, const std::string& reference_point_frame,
                 const ros::Time& time, const ros::Duration& averaging_interval,
                 geometry_msgs::Twist& twist) const
{

  ros::Time latest_time, target_time;
  getLatestCommonTime(observation_frame, tracking_frame, latest_time, NULL); ///\TODO check time on reference point too

  if (ros::Time() == time)
    target_time = latest_time;
  else
    target_time = time;

  ros::Time end_time = std::min(target_time + averaging_interval *0.5 , latest_time);

  ros::Time start_time = std::max(ros::Time().fromSec(.00001) + averaging_interval, end_time) - averaging_interval;  // don't collide with zero
  ros::Duration corrected_averaging_interval = end_time - start_time; //correct for the possiblity that start time was truncated above.
  geometry_msgs::TransformStamped start_msg, end_msg;
  start_msg = tf2_->lookupTransform(observation_frame, tracking_frame, start_time);
  end_msg = tf2_->lookupTransform(observation_frame, tracking_frame, end_time);

  tf2::Stamped< tf2::Transform > start,end;
  tf2::fromMsg(start_msg,start);
  tf2::fromMsg(end_msg,end);

  tf2::Matrix3x3 temp = start.getBasis().inverse() * end.getBasis();
  tf2::Quaternion quat_temp;
  temp.getRotation(quat_temp);
  tf2::Vector3 o = start.getBasis() * quat_temp.getAxis();
  tfScalar ang = quat_temp.getAngle();

  double delta_x = end.getOrigin().getX() - start.getOrigin().getX();
  double delta_y = end.getOrigin().getY() - start.getOrigin().getY();
  double delta_z = end.getOrigin().getZ() - start.getOrigin().getZ();


  tf2::Vector3 twist_vel ((delta_x)/corrected_averaging_interval.toSec(),
                       (delta_y)/corrected_averaging_interval.toSec(),
                       (delta_z)/corrected_averaging_interval.toSec());
  tf2::Vector3 twist_rot = o * (ang / corrected_averaging_interval.toSec());


  // This is a twist w/ reference frame in observation_frame  and reference point is in the tracking_frame at the origin (at start_time)


  //correct for the position of the reference frame
  tf2::Stamped< tf2::Transform > inverse;
  tf2::fromMsg(tf2_->lookupTransform(reference_frame,tracking_frame,  target_time),inverse);
  tf2::Vector3 out_rot = inverse.getBasis() * twist_rot;
  tf2::Vector3 out_vel = inverse.getBasis()* twist_vel + inverse.getOrigin().cross(out_rot);


  //Rereference the twist about a new reference point
  // Start by computing the original reference point in the reference frame:
  tf2::Stamped<tf2::Vector3> rp_orig(tf2::Vector3(0,0,0), target_time, tracking_frame);
  geometry_msgs::TransformStamped reference_frame_trans;
  tf2::fromMsg(tf2_->lookupTransform(reference_frame,rp_orig.frame_id_,rp_orig.stamp_),reference_frame_trans);

  geometry_msgs::PointStamped rp_orig_msg;
  tf2::toMsg(rp_orig,rp_orig_msg);
  tf2::doTransform(rp_orig_msg, rp_orig_msg, reference_frame_trans);

  // convert the requrested reference point into the right frame
  tf2::Stamped<tf2::Vector3> rp_desired(reference_point, target_time, reference_point_frame);
  geometry_msgs::PointStamped rp_desired_msg;
  tf2::toMsg(rp_desired,rp_desired_msg);
  tf2::doTransform(rp_desired_msg, rp_desired_msg, reference_frame_trans);
  // compute the delta
  tf2::Vector3 delta = rp_desired - rp_orig;
  // Correct for the change in reference point
  out_vel = out_vel + out_rot * delta;
  // out_rot unchanged

  /*
    printf("KDL: Rotation %f %f %f, Translation:%f %f %f\n",
         out_rot.x(),out_rot.y(),out_rot.z(),
         out_vel.x(),out_vel.y(),out_vel.z());
  */

  twist.linear.x =  out_vel.x();
  twist.linear.y =  out_vel.y();
  twist.linear.z =  out_vel.z();
  twist.angular.x =  out_rot.x();
  twist.angular.y =  out_rot.y();
  twist.angular.z =  out_rot.z();

};


bool TebLocalPlannerROS::transformHumanPlan(
    const tf2_ros::Buffer &tf2, const geometry_msgs::PoseStamped &robot_pose,
    const costmap_2d::Costmap2D &costmap, const std::string &global_frame,
    const std::vector<geometry_msgs::PoseWithCovarianceStamped> &human_plan,
    HumanPlanCombined &transformed_human_plan_combined,
    geometry_msgs::TwistStamped &transformed_human_twist,
    tf2::Stamped<tf2::Transform> *tf_human_plan_to_global) const {
  try {
    if (human_plan.empty()) {
      ROS_ERROR("Received human plan with zero length");
      return false;
    }

    // get human_plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped human_plan_to_global_transform;
    // tf.waitForTransform(global_frame, human_plan.front().header.frame_id,
                        // ros::Time(0), ros::Duration(0.5));
    human_plan_to_global_transform = tf2.lookupTransform(global_frame, human_plan.front().header.frame_id,
                                                                        ros::Time(0),ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > human_plan_to_global_transform_;
    tf2::fromMsg(human_plan_to_global_transform,human_plan_to_global_transform_);

    // transform the full plan to local planning frame
    std::vector<geometry_msgs::PoseStamped> transformed_human_plan;
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    geometry_msgs::PoseStamped transformed_pose;
    tf2::Transform tf_pose;
    for (auto &human_pose : human_plan) {
      tf2::fromMsg(human_pose.pose.pose, tf_pose);
      tf_pose_stamped.setData(human_plan_to_global_transform_ * tf_pose);
      tf_pose_stamped.stamp_ = human_plan_to_global_transform_.stamp_;
      tf_pose_stamped.frame_id_ = global_frame;
      tf2::toMsg(tf_pose_stamped, transformed_pose);

      transformed_human_plan.push_back(transformed_pose);
    }

    // transform human twist to local planning frame
    geometry_msgs::Twist human_to_global_twist;
    lookupTwist(global_frame, transformed_human_twist.header.frame_id,
                   ros::Time(0), ros::Duration(0.1), human_to_global_twist);
    transformed_human_twist.twist.linear.x -= human_to_global_twist.linear.x;
    transformed_human_twist.twist.linear.y -= human_to_global_twist.linear.y;
    transformed_human_twist.twist.angular.z -= human_to_global_twist.angular.z;

    double dist_threshold =
        std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0) *
        0.85;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double x_diff, y_diff, sq_dist;

    // get first point of human plan within threshold distance from robot
    int start_index = transformed_human_plan.size(), end_index = 0;
    for (int i = 0; i < transformed_human_plan.size(); i++) {
      x_diff = robot_pose.pose.position.x -
               transformed_human_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y -
               transformed_human_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        start_index = i;
        break;
      }
    }

    // now get last point of human plan withing threshold distance from robot
    for (int i = (transformed_human_plan.size() - 1); i >= 0; i--) {
      x_diff = robot_pose.pose.position.x -
               transformed_human_plan[i].pose.position.x;
      y_diff = robot_pose.pose.position.y -
               transformed_human_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      if (sq_dist < sq_dist_threshold) {
        end_index = i;
        break;
      }
    }

    // ROS_INFO("start: %d, end: %d, full: %ld", start_index, end_index,
    // transformed_human_plan.size());
    transformed_human_plan_combined.plan_before.clear();
    transformed_human_plan_combined.plan_to_optimize.clear();
    transformed_human_plan_combined.plan_after.clear();
    for (int i = 0; i < transformed_human_plan.size(); i++) {
      if (i < start_index) {
        transformed_human_plan_combined.plan_before.push_back(
            transformed_human_plan[i]);
      } else if (i >= start_index && i <= end_index) {
        transformed_human_plan_combined.plan_to_optimize.push_back(
            transformed_human_plan[i]);
      } else if (i > end_index) {
        transformed_human_plan_combined.plan_after.push_back(
            transformed_human_plan[i]);
      } else {
        ROS_ERROR("Transform human plan indexing error");
      }
    }

    if (tf_human_plan_to_global)
      *tf_human_plan_to_global = human_plan_to_global_transform_;
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (human_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(), (unsigned int)human_plan.size(),
                human_plan.front().header.frame_id.c_str());

    return false;
  }

  return true;
}

bool TebLocalPlannerROS::transformHumanPose(
    const tf2_ros::Buffer &tf2, const std::string &global_frame,
    geometry_msgs::PoseWithCovarianceStamped &human_pose,
    geometry_msgs::PoseStamped &transformed_human_pose) const {
  try {
    // get human_pose_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped human_plan_to_global_transform;
    // tf.waitForTransform(global_frame, human_pose.header.frame_id, ros::Time(0),
                        // ros::Duration(0.5));
    human_plan_to_global_transform = tf2.lookupTransform(global_frame, human_pose.header.frame_id, ros::Time(0),
                       ros::Duration(0.5));
    tf2::Stamped< tf2::Transform > human_plan_to_global_transform_;
    tf2::fromMsg(human_plan_to_global_transform,human_plan_to_global_transform_);

    // transform human pose to local planning frame
    tf2::Stamped<tf2::Transform> tf_pose_stamped;
    tf2::Transform tf_pose;
    tf2::fromMsg(human_pose.pose.pose, tf_pose);
    tf_pose_stamped.setData(human_plan_to_global_transform_ * tf_pose);
    tf_pose_stamped.stamp_ = human_plan_to_global_transform_.stamp_;
    tf_pose_stamped.frame_id_ = global_frame;
    tf2::toMsg(tf_pose_stamped, transformed_human_pose);
  } catch (tf::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  } catch (tf::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    ROS_ERROR("Global Frame: %s Pose Frame %s\n", global_frame.c_str(),
              human_pose.header.frame_id.c_str());
    return false;
  }

  return true;
}

double TebLocalPlannerROS::estimateLocalGoalOrientation(
    const std::vector<geometry_msgs::PoseStamped> &global_plan,
    const tf2::Stamped<tf2::Transform> &local_goal, int current_goal_idx,
    const tf2::Stamped<tf2::Transform> &tf_plan_to_global,
    int moving_average_length) const {
  int n = (int)global_plan.size();

  // check if we are near the global goal already
  if (current_goal_idx > n - moving_average_length - 2) {
    if (current_goal_idx >= n - 1) // we've exactly reached the goal
    {
      return tf2::impl::getYaw(local_goal.getRotation());
    } else {
      tf2::Quaternion global_orientation;
      tf2::fromMsg(global_plan.back().pose.orientation,
                            global_orientation);
      return tf2::impl::getYaw(tf_plan_to_global.getRotation() * global_orientation);
    }
  }

  // reduce number of poses taken into account if the desired number of poses
  // is not available
  moving_average_length =
      std::min(moving_average_length,
               n - current_goal_idx - 1); // maybe redundant, since we have
                                          // checked the vicinity of the goal
                                          // before

  std::vector<double> candidates;
  tf2::Stamped<tf2::Transform> tf_pose_k = local_goal;
  tf2::Stamped<tf2::Transform> tf_pose_kp1;

  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i) {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::PoseStamped &pose = global_plan.at(i + 1);
    tf2::fromMsg(pose, tf_pose_kp1);
    tf_pose_kp1.setData(tf_plan_to_global * tf_pose_kp1);

    // calculate yaw angle
    candidates.push_back(std::atan2(
        tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
        tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX()));

    if (i < range_end - 1)
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}

void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
{
  // Limit translational velocity for forward driving
    if (vx > max_vel_x)
      vx = max_vel_x;

    if (vy > max_vel_y)
      vy = max_vel_y;

    else if (vy < -max_vel_y)
      vy = -max_vel_y;

    // if (v < -max_vel_x_backwards) {
    //   v = -max_vel_x_backwards;
    // }
    // else if (v > -min_vel_x_backwards) {
    //   v = -min_vel_x_backwards;
    // }


  // Limit angular velocity
  if (omega > max_vel_theta)
    omega = max_vel_theta;
  else if (omega < -max_vel_theta)
    omega = -max_vel_theta;

  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
  vx = -max_vel_x_backwards;

  // if (omega > 0.0) {
  //   if (omega > max_vel_theta) {
  //     omega = max_vel_theta;
  //   } else if (omega < min_vel_theta) {
  //     omega = min_vel_theta;
  //   }
  // } else if (omega < 0.0) {
  //   if (omega < -max_vel_theta) {
  //     omega = -max_vel_theta;
  //   } else if (omega > -min_vel_theta) {
  //     omega = -min_vel_theta;
  //   }
  // }


  // slow change of direction in angular velocity
  if (cfg_.optim.disable_rapid_omega_chage) {
    if (std::signbit(omega) != std::signbit(last_omega_)) {
      // signs are changed
      auto now = ros::Time::now();
      if ((now - last_omega_sign_change_).toSec() <
          cfg_.optim.omega_chage_time_seperation) {
        // do not allow sign change
        omega = std::copysign(min_vel_theta, omega);
      }
      last_omega_sign_change_ = now;
      last_omega_ = omega;
    }
  }
}

double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(
    double v, double omega, double wheelbase, double min_turning_radius) const {
  if (omega == 0 || v == 0)
    return 0;

  double radius = v / omega;

  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius;

  return std::atan(wheelbase / radius);
}

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}



 void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();

     // reduced horizon backup mode
    if (cfg_.trajectory.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.trajectory.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.trajectory.shrink_horizon_min_duration);


         // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;

         if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }

         // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }

 }

void TebLocalPlannerROS::customObstacleCB(
    const teb_local_planner::ObstacleMsg::ConstPtr &obst_msg) {
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(
    const ros::NodeHandle &nh) {
  std::string model_name;
  if (!nh.getParam("footprint_model/type", model_name)) {
    ROS_INFO("No robot footprint model specified for trajectory optimization. "
             "Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }

  // point
  if (model_name.compare("point") == 0) {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>();
  }

  // circular
  if (model_name.compare("circular") == 0) {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius)) {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for "
                       "trajectory optimization, since param '"
                       << nh.getNamespace()
                       << "/footprint_model/radius' does not exist. Using "
                          "point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: "
                    << radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }

  // line
  if (model_name.compare("line") == 0) {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") ||
        !nh.hasParam("footprint_model/line_end")) {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory "
                       "optimization, since param '"
                       << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' "
                          "do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2) {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory "
                       "optimization, since param '"
                       << nh.getNamespace()
                       << "/footprint_model/line_start' and/or '.../line_end' "
                          "do not contain x and y coordinates (2D). Using "
                          "point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }

    ROS_INFO_STREAM("Footprint model 'line' (line_start: ["
                    << line_start[0] << "," << line_start[1]
                    << "]m, line_end: [" << line_end[0] << "," << line_end[1]
                    << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(
        Eigen::Map<const Eigen::Vector2d>(line_start.data()),
        Eigen::Map<const Eigen::Vector2d>(line_end.data()));
  }

  // two circles
  if (model_name.compare("two_circles") == 0) {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") ||
        !nh.hasParam("footprint_model/front_radius") ||
        !nh.hasParam("footprint_model/rear_offset") ||
        !nh.hasParam("footprint_model/rear_radius")) {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for "
                       "trajectory optimization, since params '"
                       << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', "
                          "'.../rear_offset' and '.../rear_radius' do not "
                          "exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: "
                    << front_offset << "m, front_radius: " << front_radius
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: "
                    << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(
        front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0) {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc)) {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for "
                       "trajectory optimization, since param '"
                       << nh.getNamespace()
                       << "/footprint_model/vertices' does not exist. Using "
                          "point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      try {
        Point2dContainer polygon = makeFootprintFromXMLRPC(
            footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM(
            "Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } catch (const std::exception &ex) {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for "
                         "trajectory optimization: "
                         << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    } else {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for "
                       "trajectory optimization, since param '"
                       << nh.getNamespace()
                       << "/footprint_model/vertices' does not define an array "
                          "of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
  }

  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '"
                  << nh.getNamespace()
                  << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}

Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(
    XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name) {
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      footprint_xmlrpc.size() < 3) {
    ROS_FATAL("The footprint must be specified as list of lists on the "
              "parameter server, %s was specified as %s",
              full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error(
        "The footprint must be specified as list of lists on the parameter "
        "server with at least "
        "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  Point2dContainer footprint;
  Eigen::Vector2d pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y
    // coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        point.size() != 2) {
      ROS_FATAL("The footprint (parameter %s) must be specified as list of "
                "lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of "
                "that form.",
                full_param_name.c_str());
      throw std::runtime_error("The footprint must be specified as list of "
                               "lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this "
                               "spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double
TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value,
                                        const std::string &full_param_name) {
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string &value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be "
              "numbers. Found value %s.",
              full_param_name.c_str(), value_string.c_str());
    throw std::runtime_error(
        "Values in the footprint specification must be numbers");
  }
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value)
                                                         : (double)(value);
}

void TebLocalPlannerROS::resetHumansPrediction() {
  std_srvs::Empty empty_service;
  ROS_INFO("Resetting human pose prediction");
  if (!reset_humans_prediction_client_ ||
      !reset_humans_prediction_client_.call(empty_service)) {
    ROS_WARN_THROTTLE(
        THROTTLE_RATE,
        "Failed to call %s service, is human prediction server running?",
        PREDICT_SERVICE_NAME);

    // re-initialize the service
    // reset_humans_prediction_client_ =
    //     nh.serviceClient<std_srvs::Empty>(RESET_PREDICTION_SERVICE_NAME,
    //     true);
  }
}

bool TebLocalPlannerROS::optimizeStandalone(
    teb_local_planner::Optimize::Request &req,
    teb_local_planner::Optimize::Response &res) {
  ROS_INFO("optimize service called");
  auto start_time = ros::Time::now();

  // check if plugin initialized
  if (!initialized_) {
    res.success = false;
    res.message = "planner has not been initialized";
    return true;
  }

  auto trfm_start_time = ros::Time::now();
  // get robot pose from the costmap
  // tf2::Stamped<tf2::Transform> robot_pose_tf;
  geometry_msgs::PoseStamped robot_pose_tf;
  costmap_ros_->getRobotPose(robot_pose_tf);

  // transform global plan to the frame of local costmap
  ROS_INFO("transforming robot global plans");
  PlanCombined transformed_plan_combined;
  int goal_idx;
  tf2::Stamped<tf2::Transform> tf_robot_plan_to_global;
  tf2::Stamped<tf2::Transform> robot_pose_tf_;
  tf2::fromMsg(robot_pose_tf,robot_pose_tf_);
  if (!transformGlobalPlan(
          *tf2_, req.robot_plan.poses, robot_pose_tf_, *costmap_, global_frame_,
          cfg_.trajectory.max_global_plan_lookahead_dist, transformed_plan_combined,
          &goal_idx, &tf_robot_plan_to_global)) {
    res.success = false;
    res.message = "Could not transform the global plan to the local frame";
    return true;
  }
  auto &transformed_plan = transformed_plan_combined.plan_to_optimize;
  ROS_INFO("transformed plan contains %ld points (out of %ld)",
           transformed_plan.size(), req.robot_plan.poses.size());

  // check if the transformed robot plan is empty
  if (transformed_plan.empty()) {
    res.success = false;
    res.message = "Robot's transformed plan is empty";
    return true;
  }
  auto trfm_time = ros::Time::now() - trfm_start_time;

  // update obstacles container
  auto cc_start_time = ros::Time::now();
  obstacles_.clear();
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  updateObstacleContainerWithCustomObstacles();
  auto cc_time = ros::Time::now() - cc_start_time;

  // update via-points container
  auto via_start_time = ros::Time::now();
  updateViaPointsContainer(transformed_plan,
                           cfg_.trajectory.global_plan_viapoint_sep);
  auto via_time = ros::Time::now() - via_start_time;

  // do not allow config changes from now until end of optimization
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // update humans
  auto human_start_time = ros::Time::now();

  HumanPlanVelMap transformed_human_plan_vel_map;
  std::vector<HumanPlanCombined> transformed_human_plans;
  tf2::Stamped<tf2::Transform> tf_human_plan_to_global;
  for (auto human_path : req.human_path_array.paths) {
    HumanPlanCombined human_plan_combined;
    geometry_msgs::TwistStamped transformed_vel;
    transformed_vel.header.frame_id = global_frame_;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> human_path_cov;
    for (auto human_pose : human_path.path.poses) {
      geometry_msgs::PoseWithCovarianceStamped human_pos_cov;
      human_pos_cov.header = human_pose.header;
      human_pos_cov.pose.pose = human_pose.pose;
      human_path_cov.push_back(human_pos_cov);
    }
    ROS_INFO("transforming human %ld plan", human_path.id);
    if (!transformHumanPlan(*tf2_, robot_pose_tf, *costmap_, global_frame_,
                            human_path_cov, human_plan_combined,
                            transformed_vel, &tf_human_plan_to_global)) {
      res.success = false;
      res.message = "could not transform human" +
                    std::to_string(human_path.id) + " plan to the local frame";
      return true;
    }
    auto transformed_plan_size = human_plan_combined.plan_before.size() +
                                 human_plan_combined.plan_to_optimize.size() +
                                 human_plan_combined.plan_after.size();
    ROS_INFO("transformed human %ld plan contains %ld (before %ld, "
             "to-optimize %ld, after %ld) points (out of %ld (%ld))",
             human_path.id, transformed_plan_size,
             human_plan_combined.plan_before.size(),
             human_plan_combined.plan_to_optimize.size(),
             human_plan_combined.plan_after.size(), human_path_cov.size(),
             human_path.path.poses.size());
    // TODO: check for empty human transformed plan

    human_plan_combined.id = human_path.id;
    transformed_human_plans.push_back(human_plan_combined);

    PlanStartVelGoalVel plan_start_vel_goal_vel;
    plan_start_vel_goal_vel.plan = human_plan_combined.plan_to_optimize;
    plan_start_vel_goal_vel.start_vel = transformed_vel.twist;
    if (human_plan_combined.plan_after.size() > 0) {
      plan_start_vel_goal_vel.goal_vel = transformed_vel.twist;
    }
    transformed_human_plan_vel_map[human_plan_combined.id] =
        plan_start_vel_goal_vel;
  }

  updateHumanViaPointsContainers(transformed_human_plan_vel_map,
                                 cfg_.trajectory.global_plan_viapoint_sep);
  auto human_time = ros::Time::now() - human_start_time;

  // now perform the actual planning
  auto plan_start_time = ros::Time::now();
  geometry_msgs::Twist robot_vel_twist;
  bool success = planner_->plan(transformed_plan, &robot_vel_,
                                cfg_.goal_tolerance.free_goal_vel,
                                &transformed_human_plan_vel_map);
  if (!success) {
    planner_->clearPlanner();
    res.success = false;
    res.message =
        "planner was not able to obtain a local plan for the current setting";
    return true;
  }
  auto plan_time = ros::Time::now() - plan_start_time;

  // now visualize everything
  auto viz_start_time = ros::Time::now();
  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  visualization_->publishHumanGlobalPlans(transformed_human_plans);

  PlanTrajCombined plan_traj_combined;
  plan_traj_combined.plan_before = transformed_plan_combined.plan_before;
  planner_->getFullTrajectory(plan_traj_combined.optimized_trajectory);
  plan_traj_combined.plan_after = transformed_plan_combined.plan_after;
  visualization_->publishTrajectory(plan_traj_combined);

  std::vector<HumanPlanTrajCombined> human_plans_traj_array;
  for (auto &human_plan_combined : transformed_human_plans) {
    HumanPlanTrajCombined human_plan_traj_combined;
    human_plan_traj_combined.id = human_plan_combined.id;
    human_plan_traj_combined.plan_before = human_plan_combined.plan_before;
    planner_->getFullHumanTrajectory(
        human_plan_traj_combined.id,
        human_plan_traj_combined.optimized_trajectory);
    human_plan_traj_combined.plan_after = human_plan_combined.plan_after;
    human_plans_traj_array.push_back(human_plan_traj_combined);
  }
  visualization_->publishHumanTrajectories(human_plans_traj_array);
  auto viz_time = ros::Time::now() - viz_start_time;

  res.success = true;
  res.message = "planning successful";
  geometry_msgs::Twist cmd_vel;

  // check feasibility of robot plan
  auto fsb_start_time = ros::Time::now();
  bool feasible = planner_->isTrajectoryFeasible(
      costmap_model_.get(), footprint_spec_, robot_inscribed_radius_,
      robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible) {
    res.message += "\nhowever, trajectory is not feasible";
  }
  auto fsb_time = ros::Time::now() - fsb_start_time;

  // get the velocity command for this sampling interval
  auto vel_start_time = ros::Time::now();
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.angular.z)) {
    res.message += feasible ? "\nhowever," : "\nand";
    res.message += " velocity command is invalid";
  }

  // clear the planner only after getting the velocity command
  planner_->clearPlanner();

  // saturate velocity
  saturateVelocity(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.max_vel_x,
                   cfg_.robot.min_vel_x, cfg_.robot.max_vel_theta,
                   cfg_.robot.min_vel_theta, cfg_.robot.max_vel_x_backwards,
                   cfg_.robot.min_vel_x_backwards);
  auto vel_time = ros::Time::now() - vel_start_time;

  auto total_time = ros::Time::now() - start_time;

  res.message += "\ncompute velocity times:";
  res.message +=
      "\n\ttotal time                  " + std::to_string(total_time.toSec()) +
      "\n\ttransform time              " + std::to_string(trfm_time.toSec()) +
      "\n\tcostmap convert time        " + std::to_string(cc_time.toSec()) +
      "\n\tvia points time             " + std::to_string(via_time.toSec()) +
      "\n\thuman time                  " + std::to_string(human_time.toSec()) +
      "\n\tplanning time               " + std::to_string(plan_time.toSec()) +
      "\n\tplan feasibility check time " + std::to_string(fsb_time.toSec()) +
      "\n\tvelocity extract time       " + std::to_string(vel_time.toSec()) +
      "\n\tvisualization publish time  " + std::to_string(viz_time.toSec()) +
      "\n=================================";
  return true;
}

bool TebLocalPlannerROS::setApproachID(
    teb_local_planner::Approach::Request &req,
    teb_local_planner::Approach::Response &res) {
  if (cfg_.planning_mode == 2) {
    cfg_.approach.approach_id = req.human_id;
    res.message +=
        "Approach ID set to " + std::to_string(cfg_.approach.approach_id);
    res.success = true;
  } else {
    cfg_.approach.approach_id = -1;
    res.message = "No approach ID set, planner is not running in approach mode";
    res.success = false;
  }
  return true;
}

} // end namespace teb_local_planner
