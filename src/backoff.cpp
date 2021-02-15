/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 * Author: Phani Teja Singamaneni
 *********************************************************************/

 #include <teb_local_planner/backoff.h>
 #define NODE_NAME "backoff_recovery"
 #define ROBOT_FRAME_ID "base_footprint"
 #define MAP_FRAME_ID "map"
 #define GET_PLAN_SRV_NAME "/move_base_node/GlobalPlanner/make_plan"


 namespace backoff {
 // empty constructor and destructor
 Backoff::Backoff(): costmap_ros_(NULL) /*ac("move_base", false)*/ {}
 Backoff::~Backoff() {}

 void Backoff::initialize(costmap_2d::Costmap2DROS* costmap_ros) {
   // get private node handle
   ros::NodeHandle nh("~/");
   costmap_ros_ = costmap_ros;
   costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

   current_goal_sub_ = nh.subscribe("/move_base_node/current_goal", 1, &Backoff::currentgoalCB, this);
   // goal_result_sub_ = nh.subscribe("/move_base/result", 1, &Backoff::goalresultCB, this);
   goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",100);
   get_plan_client_ = nh.serviceClient<nav_msgs::GetPlan>(GET_PLAN_SRV_NAME, true);
   reset_ = true;
   exec_goal = false;
   NEW_GOAL = false;
   last_flag = false;

  ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
 }

 void Backoff::currentgoalCB(const geometry_msgs::PoseStamped::ConstPtr &goal){
   auto tmp_goal = *goal;
   if(reset_){
     current_goal_ = tmp_goal;
     NEW_GOAL = false;
   }
   else if(current_goal_.header.stamp!=tmp_goal.header.stamp && exec_goal){
     if(!last_flag){
       current_goal_ = tmp_goal;
       NEW_GOAL = true;
     }
     last_flag = false;
   }
 }

 // void Backoff::goalresultCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &gres){
 //   g_res_ = *gres;
 //   if(g_res_.status.status==3){
 //     exec_goal = false;
 //     std::cout << "I am here" << '\n';
 //   }
 // }

 bool Backoff::recovery(){
   reset_ = false;
   NEW_GOAL = false;
   // get robot pose
   tf::StampedTransform robot_to_map_tf;
   bool transform_found = false;
   try {
     tf_.lookupTransform(MAP_FRAME_ID, ROBOT_FRAME_ID, ros::Time(0),
                         robot_to_map_tf);
     transform_found = true;
   } catch (tf::LookupException &ex) {
     ROS_ERROR_NAMED(NODE_NAME, "No Transform available Error: %s\n",
                     ex.what());
   } catch (tf::ConnectivityException &ex) {
     ROS_ERROR_NAMED(NODE_NAME, "Connectivity Error: %s\n", ex.what());
   } catch (tf::ExtrapolationException &ex) {
     ROS_ERROR_NAMED(NODE_NAME, "Extrapolation Error: %s\n", ex.what());
   }
   catch(...){
     std::cout << "Something else" << '\n';
   }
   auto now = ros::Time::now();

   if(transform_found){

     tf::Transform start_pose_tr ,behind_tr, right_tr, left_tr, front_tr;
     start_pose_tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
     start_pose_tr.setRotation(tf::createQuaternionFromYaw(0.0));
     start_pose_tr = robot_to_map_tf * start_pose_tr;

     behind_tr.setOrigin(tf::Vector3(-0.5, 0.0, 0.0));
     behind_tr.setRotation(tf::createQuaternionFromYaw(0.0));
     behind_tr = robot_to_map_tf * behind_tr;

     right_tr.setOrigin(tf::Vector3(0.0, -1.0, 0.0));
     right_tr.setRotation(tf::createQuaternionFromYaw(0.0));
     right_tr = robot_to_map_tf * right_tr;

     left_tr.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
     left_tr.setRotation(tf::createQuaternionFromYaw(0.0));
     left_tr = robot_to_map_tf * left_tr;

     tf::transformTFToMsg(behind_tr, behind_pose);
     tf::transformTFToMsg(right_tr, right_pose);
     tf::transformTFToMsg(left_tr, left_pose);

     // Another working way but throws errors
     //
     // nav_msgs::GetPlan get_plan_srv;
     // get_plan_srv.request.start.header.frame_id = MAP_FRAME_ID;
     // get_plan_srv.request.start.header.stamp = now;
     // tf::poseTFToMsg(robot_to_map_tf, get_plan_srv.request.start.pose);
     //
     // get_plan_srv.request.goal.header.stamp = now;
     // get_plan_srv.request.goal.header.frame_id = MAP_FRAME_ID;

     // make plan for robot
     // if (get_plan_client_) {
     //   get_plan_srv.request.goal.pose.position.x = right_pose.translation.x;
     //   get_plan_srv.request.goal.pose.position.y = right_pose.translation.y;
     //   get_plan_srv.request.goal.pose.position.z = right_pose.translation.z;
     //   get_plan_srv.request.goal.pose.orientation = right_pose.rotation;
     //   try{
     //     get_plan_client_.call(get_plan_srv);
     //
     //     if (get_plan_srv.response.plan.poses.size() > 0) {
     //      // ROS_INFO("feasible !!");
     //      goal_pub_.publish(get_plan_srv.request.goal);
     //      return true;
     //     }
     //     get_plan_srv.request.goal.pose.position.x = left_pose.translation.x;
     //     get_plan_srv.request.goal.pose.position.y = left_pose.translation.y;
     //     get_plan_srv.request.goal.pose.position.z = left_pose.translation.z;
     //     get_plan_srv.request.goal.pose.orientation = left_pose.rotation;
     //   }
     //   catch (ros::Exception &e){
     //     // ROS_ERROR("Error occured: %s ", e.what());
     //     ROS_DEBUG("Right not feasible");
     //   }
     //
     //   try {
     //     get_plan_client_.call(get_plan_srv);
     //
     //     if (get_plan_srv.response.plan.poses.size() > 0) {
     //      // ROS_INFO("feasible !!");
     //      goal_pub_.publish(get_plan_srv.request.goal);
     //      return true;
     //     }
     //     get_plan_srv.request.goal.pose.position.x = behind_pose.translation.x;
     //     get_plan_srv.request.goal.pose.position.y = behind_pose.translation.y;
     //     get_plan_srv.request.goal.pose.position.z = behind_pose.translation.z;
     //     get_plan_srv.request.goal.pose.orientation = behind_pose.rotation;
     //   }
     //   catch (ros::Exception &e){
     //     ROS_DEBUG("left not feasible");
     //   }
     //
     //   if (get_plan_client_.call(get_plan_srv)) {
     //     if (get_plan_srv.response.plan.poses.size() > 0) {
     //      ROS_INFO("Going Back !!");
     //      goal_pub_.publish(get_plan_srv.request.goal);
     //      return false;
     //     }
     //   }
     //
     //   else {
     //     ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service",
     //                    GET_PLAN_SRV_NAME);
     //   }
     //
     // } else {
     //   ROS_WARN_NAMED(NODE_NAME,
     //                  "%s service does not exist, re-trying to subscribe",
     //                  GET_PLAN_SRV_NAME);
     //   ros::NodeHandle nh("~/");
     //   get_plan_client_ = nh.serviceClient<nav_msgs::GetPlan>(GET_PLAN_SRV_NAME, true);
     // }
 }
    unsigned int mx,my;
    goal_.header.frame_id = MAP_FRAME_ID;
    goal_.header.stamp = now;

    if(costmap_->worldToMap(right_pose.translation.x,right_pose.translation.y,mx,my)){
      auto cost=costmap_->getCost(mx,my);
      if(cost == costmap_2d::FREE_SPACE && !exec_goal){
        goal_.pose.position.x = right_pose.translation.x;
        goal_.pose.position.y = right_pose.translation.y;
        goal_.pose.orientation = right_pose.rotation;
        goal_pub_.publish(goal_);
        exec_goal = true;
        last_flag = true;
        return true;
      }
    }
    if(costmap_->worldToMap(left_pose.translation.x,left_pose.translation.y,mx,my)){
      auto cost=costmap_->getCost(mx,my);
      if(cost == costmap_2d::FREE_SPACE && !exec_goal){
        goal_.pose.position.x = left_pose.translation.x;
        goal_.pose.position.y = left_pose.translation.y;
        goal_.pose.orientation = left_pose.rotation;
        goal_pub_.publish(goal_);
        exec_goal = true;
        last_flag = true;
        return true;
      }
    }

    if(costmap_->worldToMap(behind_pose.translation.x,behind_pose.translation.y,mx,my)){
      auto cost=costmap_->getCost(mx,my);
      if(cost == costmap_2d::FREE_SPACE){
        goal_.pose.position.x = behind_pose.translation.x;
        goal_.pose.position.y = behind_pose.translation.y;
        goal_.pose.orientation = behind_pose.rotation;
        goal_pub_.publish(goal_);
        exec_goal = false;
        return false;
      }
    }

    return false;
 }

 bool Backoff::setback_goal(){
   goal_ = current_goal_;
   goal_.header.stamp=ros::Time::now();
   goal_pub_.publish(goal_);
   reset_ = true;
   return true;
 }

} // end namespace backoff
