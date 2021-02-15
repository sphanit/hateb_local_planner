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
 #ifndef BACKOFF_H_
 #define BACKOFF_H_

 #include <ros/ros.h>
 #include <dynamic_reconfigure/server.h>
 #include <tf/transform_listener.h>
 #include <std_srvs/SetBool.h>
 #include <std_srvs/Trigger.h>
 #include <move_base_msgs/MoveBaseAction.h>
 #include <actionlib/client/simple_action_client.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <nav_msgs/GetPlan.h>
 #include <move_base_msgs/MoveBaseActionResult.h>

 // #include <geometry_msgs/PoseStamped.h>

 namespace backoff {
 class Backoff {
 public:
   Backoff();
   ~Backoff();
   void initialize(costmap_2d::Costmap2DROS* costmap_ros);
   bool recovery();
   bool setback_goal();
   bool NEW_GOAL;

 private:
   geometry_msgs::PoseStamped goal_;

   tf::TransformListener tf_;

   costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
   costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)

   ros::Subscriber current_goal_sub_,goal_result_sub_;
   ros::Publisher goal_pub_;
   ros::ServiceClient get_plan_client_;
   geometry_msgs::PoseStamped current_goal_;
   // move_base_msgs::MoveBaseActionResult g_res_;
   geometry_msgs::Transform behind_pose, right_pose, left_pose;
   bool reset_;
   bool exec_goal, last_flag;

   void currentgoalCB(const geometry_msgs::PoseStamped::ConstPtr &goal);
   // void goalresultCB(const move_base_msgs::MoveBaseActionResult::ConstPtr &gres);

};
}
#endif // BACKOFF_H_
