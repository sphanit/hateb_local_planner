/*********************************************************************
 * Author: Phani Teja S
 *********************************************************************/
 #ifndef STATIC_PLAN_VISUALIZER_H_
 #define STATIC_PLAN_VISUALIZER_H_

 #include <ros/ros.h>

 #include <tf2_ros/transform_listener.h>
 #include <tf/tf.h>
 #include <tf2/utils.h>
 #include <tf2/impl/utils.h>
 #include <tf2/convert.h>
 #include <tf2_ros/buffer.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <tf/transform_listener.h>

 #include <nav_msgs/GetPlan.h>
 #include <nav_msgs/Path.h>
 #include <hanp_msgs/TrackedHumans.h>
 #include <hanp_msgs/HumanPathArray.h>
 #include <hanp_msgs/HumanPath.h>
 #include <hanp_msgs/HumanTrajectoryArray.h>
 #include <hanp_msgs/TrackedSegmentType.h>

 #include <teb_local_planner/Optimize.h>
 #include <std_srvs/Trigger.h>
 #include <std_srvs/TriggerRequest.h>
 #include <std_srvs/TriggerResponse.h>


namespace teb_local_planner{
   class StaticPlanVisualization{
   public:
     StaticPlanVisualization(tf2_ros::Buffer &tf2_);

     ~StaticPlanVisualization();

     void initialize();

     void UpdateStartPoses(const hanp_msgs::TrackedHumans &tracked_humans);

     void UpdateGoalsAndOptimize(const geometry_msgs::PointStamped &robot_goal_point);

     bool optimize_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


   private:
     tf2_ros::Buffer &tf_;
     tf2_ros::TransformListener tfListener_;
     bool initialized_, predict_behind_robot_, got_robot_plan, got_human_plan;
     ros::ServiceClient optimize_client, getPlan_client;
     ros::ServiceServer optimize_srv_;
     ros::Subscriber humans_sub_, robot_goal_sub_;

     tf2_ros::Buffer tfBuffer;
     std::vector<geometry_msgs::PoseStamped> humans_start_poses, humans_goals_;
     geometry_msgs::PoseStamped robot_start_pose, robot_goal_;
     geometry_msgs::TransformStamped robot_to_map_tf;
     hanp_msgs::TrackedHumans tracked_humans_;
     hanp_msgs::HumanPathArray humans_plans;
     nav_msgs::Path robot_plan;



 };// class StaticPlanVisualization
};// namespace teb_local_planner
#endif // STATIC_PLAN_VISUALIZER_H_
