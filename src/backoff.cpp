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
   count = 0;
   orient = 0;
   RAND_ROTATE=false;
   last_time = ros::Time::now();
   last_rot_time = ros::Time::now();

  ROS_DEBUG_NAMED(NODE_NAME, "node %s initialized", NODE_NAME);
 }

 void Backoff::currentgoalCB(const geometry_msgs::PoseStamped::ConstPtr &goal){
   auto tmp_goal = *goal;
   if(reset_){
     current_goal_ = tmp_goal;
     NEW_GOAL = false;
     // RAND_ROTATE = false;
     count = 0;
   }
   else if(current_goal_.header.stamp!=tmp_goal.header.stamp && exec_goal){
     if(!last_flag){
       current_goal_ = tmp_goal;
       NEW_GOAL = true;
       // RAND_ROTATE = false;
       count = 0;
       exec_goal = false;
       last_time = ros::Time::now();
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

 bool Backoff::recovery(double ang_theta){
   reset_ = false;
   NEW_GOAL = false;
   // get robot pose
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
     tf::transformTFToMsg(start_pose_tr, start_pose);

     // Another working way but throws errors
     //
     nav_msgs::GetPlan get_plan_srv;
     get_plan_srv.request.start.header.frame_id = MAP_FRAME_ID;
     get_plan_srv.request.start.header.stamp = now;
     tf::poseTFToMsg(robot_to_map_tf, get_plan_srv.request.start.pose);

     get_plan_srv.request.goal.header.stamp = now;
     get_plan_srv.request.goal.header.frame_id = MAP_FRAME_ID;

     if(abs(prev_start_pose.translation.x-start_pose.translation.x)<0.01 && abs(prev_start_pose.translation.y-start_pose.translation.y)<0.01)
       count++;
     else
       count=0;
     // std::cout << "count " <<count<< '\n';

     prev_start_pose = start_pose;

     // make plan for robot
     if (get_plan_client_) {
       behind_tr.setOrigin(tf::Vector3(-0.5, 0.0, 0.0));
       behind_tr.setRotation(tf::createQuaternionFromYaw(0.0));
       behind_tr = robot_to_map_tf * behind_tr;
       geometry_msgs::Transform temp_pose;
       tf::transformTFToMsg(behind_tr, temp_pose);

       get_plan_srv.request.goal.pose.position.x = temp_pose.translation.x;
       get_plan_srv.request.goal.pose.position.y = temp_pose.translation.y;
       get_plan_srv.request.goal.pose.position.z = temp_pose.translation.z;
       get_plan_srv.request.goal.pose.orientation = temp_pose.rotation;
       // try{
       //   get_plan_client_.call(get_plan_srv);
       //
       //   if (get_plan_srv.response.plan.poses.size() > 0) {
       //    // ROS_INFO("feasible !!");
       //    goal_pub_.publish(get_plan_srv.request.goal);
       //    return true;
       //   }
       //   get_plan_srv.request.goal.pose.position.x = left_pose.translation.x;
       //   get_plan_srv.request.goal.pose.position.y = left_pose.translation.y;
       //   get_plan_srv.request.goal.pose.position.z = left_pose.translation.z;
       //   get_plan_srv.request.goal.pose.orientation = left_pose.rotation;
       // }
       // catch (ros::Exception &e){
       //   // ROS_ERROR("Error occured: %s ", e.what());
       //   ROS_DEBUG("Right not feasible");
       // }
       //
       // try {
       //   get_plan_client_.call(get_plan_srv);
       //
       //   if (get_plan_srv.response.plan.poses.size() > 0) {
       //    // ROS_INFO("feasible !!");
       //    goal_pub_.publish(get_plan_srv.request.goal);
       //    return true;
       //   }
       //   get_plan_srv.request.goal.pose.position.x = behind_pose.translation.x;
       //   get_plan_srv.request.goal.pose.position.y = behind_pose.translation.y;
       //   get_plan_srv.request.goal.pose.position.z = behind_pose.translation.z;
       //   get_plan_srv.request.goal.pose.orientation = behind_pose.rotation;
       // }
       // catch (ros::Exception &e){
       //   ROS_DEBUG("left not feasible");
       // }

       if (get_plan_client_.call(get_plan_srv)) {
         if (get_plan_srv.response.plan.poses.size() > 0) {
          // ROS_INFO("Going Back !!");
          // goal_pub_.publish(get_plan_srv.request.goal);
          // return false;
         }
         else
          count = 10;
       }

       else {
         ROS_WARN_NAMED(NODE_NAME, "Failed to call %s service",
                        GET_PLAN_SRV_NAME);
       }

     } else {
       ROS_WARN_NAMED(NODE_NAME,
                      "%s service does not exist, re-trying to subscribe",
                      GET_PLAN_SRV_NAME);
       ros::NodeHandle nh("~/");
       get_plan_client_ = nh.serviceClient<nav_msgs::GetPlan>(GET_PLAN_SRV_NAME, true);
     }
 }
    unsigned int mx,my;
    goal_.header.frame_id = MAP_FRAME_ID;
    goal_.header.stamp = now;
    // std::cout << "I have reached to step 1" << '\n';
    if(costmap_->worldToMap(right_pose.translation.x,right_pose.translation.y,mx,my)){
      auto cost=costmap_->getCost(mx,my);
      if(cost == costmap_2d::FREE_SPACE && !exec_goal){
        goal_.pose.position.x = right_pose.translation.x;
        goal_.pose.position.y = right_pose.translation.y;
        goal_.pose.orientation = right_pose.rotation;
        goal_pub_.publish(goal_);
        exec_goal = true;
        last_flag = true;
        // std::cout << "I have reached to step 1.1" << '\n';
        return true;
      }
    }
    // std::cout << "I have reached to step 2" << '\n';

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
    // std::cout << "I have reached to step 3" << '\n';

    if(costmap_->worldToMap(behind_pose.translation.x,behind_pose.translation.y,mx,my)){
      auto cost=costmap_->getCost(mx,my);
      // std::cout << "I have reached to step 4" << '\n';

      if(cost == costmap_2d::FREE_SPACE){
        goal_.pose.position.x = behind_pose.translation.x;
        goal_.pose.position.y = behind_pose.translation.y;
        goal_.pose.orientation = behind_pose.rotation;
        // std::cout << "I have reached to step 5" << '\n';

        if(count>1){
          set_random_rotate(ang_theta);
          // std::cout << "orient" <<ang_theta<< '\n';
          count = 0;
        }

        goal_pub_.publish(goal_);
        exec_goal = false;
        return false;
      }
      else if(count>2){
        // double MIN = -0.05;
        // double MAX = 0.05;
        // orient = MIN + (double)(rand()) / ((double)(RAND_MAX/(MAX - MIN)));
        // // robot_to_map_tf
        // auto q_tr = robot_to_map_tf.getRotation();
        // geometry_msgs::Quaternion qt;
        // tf::quaternionTFToMsg(q_tr,qt);
        // auto pr_orient = tf2::getYaw(qt);
        // start_pose_tr.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        // start_pose_tr.setRotation(tf::createQuaternionFromYaw(pr_orient+orient));
        // start_pose_tr = robot_to_map_tf * start_pose_tr;
        // tf::transformTFToMsg(start_pose_tr, start_pose);
        // goal_.pose.position.x = start_pose.translation.x;
        // goal_.pose.position.y = start_pose.translation.y;
        // goal_.pose.orientation = start_pose.rotation;
        set_random_rotate(ang_theta);
        // std::cout << "orient" <<ang_theta<< '\n';
        count = 0;
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
   // RAND_ROTATE = false;
   return true;
 }


 bool Backoff::set_random_rotate(double ang_theta){
   // double MIN = -0.05;
   // double MAX = 0.05;
   // orient = MIN + (double)(rand()) / ((double)(RAND_MAX/(MAX - MIN)));
   // auto q_tr = robot_to_map_tf.getRotation();
   // geometry_msgs::Quaternion qt;
   // tf::quaternionTFToMsg(q_tr,qt);
   // auto pr_orient = tf2::getYaw(qt);
   // start_pose_tr.setRotation(tf::createQuaternionFromYaw(normalize_theta(pr_orient+orient)));

   start_pose_tr.setOrigin(tf::Vector3(0.1, 0.0, 0.0));
   start_pose_tr.setRotation(tf::createQuaternionFromYaw(0.0));
   start_pose_tr = robot_to_map_tf * start_pose_tr;
   start_pose_tr.setRotation(tf::createQuaternionFromYaw(ang_theta));
   tf::transformTFToMsg(start_pose_tr, start_pose);
   goal_.pose.position.x = start_pose.translation.x;
   goal_.pose.position.y = start_pose.translation.y;
   goal_.pose.orientation = start_pose.rotation;
   RAND_ROTATE = true;
   last_rot_time = ros::Time::now();
   goal_pub_.publish(goal_);
   ROS_INFO("Setting robot in proper orientation = %f",ang_theta);
   return true;
 }

 double Backoff::normalize_theta(double theta)
 {
   if (theta >= -M_PI && theta < M_PI)
     return theta;

   double multiplier = floor(theta / (2*M_PI));
   theta = theta - multiplier*2*M_PI;
   if (theta >= M_PI)
     theta -= 2*M_PI;
   if (theta < -M_PI)
     theta += 2*M_PI;
   return theta;
 }

 bool Backoff::check_new_goal(){
   if((ros::Time::now()-last_time).toSec() > 0.5){
    NEW_GOAL = false;
  }
  // std::cout << "(last_time-ros::Time::now()).toSec() " <<(last_time-ros::Time::now()).toSec()<< '\n';
   return NEW_GOAL;
 }

 bool Backoff::check_random_rot(){
   if((ros::Time::now()-last_rot_time).toSec() > 10.0){
    RAND_ROTATE = false;
  }
  // std::cout << "RAND_ROTATE " <<RAND_ROTATE<< '\n';
  return RAND_ROTATE;
 }

} // end namespace backoff
