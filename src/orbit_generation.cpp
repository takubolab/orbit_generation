#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_data_handler/trajectory_data_handler.h>
#include <fcsc_visual_tools/fcsc_visual_tools.h>
// #include <fcsc_moveit/smach_daihen_ur5_moveit_core.h>

using namespace std;

double deg2rad(double deg)
{
  double rad = deg * M_PI / 180.0;
  // cout << rad << endl;
  return (rad);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orbit_generation");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  std::string str("test");
  TrajectoryDataHandler handler;
  moveit_visual_tools::FCSCVisualTools object_visualizer;
  geometry_msgs::PoseStamped shelf_pose;
  std::string shelf_name("shelfA");
  std::string file_name("result");
  geometry_msgs::PoseStamped sandwich_pose;
  std::string sandwich_name;
  double roll, pitch, yaw;
  int count = 0;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  std::stringstream file_name_stream;
  std::ofstream result_file;

  time_t now = time(NULL);
  struct tm *pnow = localtime(&now);

  file_name_stream << ros::package::getPath("orbit_generation") + "/result/";
  file_name_stream << file_name << "_" << pnow->tm_year + 1900 << "-" << pnow->tm_mon + 1 << "-" << pnow->tm_mday << "-" << pnow->tm_hour << "-" << pnow->tm_min << ".csv";

  result_file.open(file_name_stream.str());
  result_file << "count"
              << ","
              << "shelf_x"
              << ","
              << "shelf_y"
              << ","
              << "x"
              << ","
              << "y"
              << ","
              << "z"
              << ","
              << "judge" << std::endl;

  
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // moveit::planning_interface::MoveGroupInterface move_group2("mobile_base");
  moveit::planning_interface::MoveGroupInterface move_group2("daisha");
  moveit::planning_interface::MoveGroupInterface move_group3("endeffector");
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // while (ros::ok()){
  //   visualization_msgs::Marker marker;
  //   // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  //   marker.header.frame_id = "/my_frame";
  //   marker.header.stamp = ros::Time::now();
  //   marker.ns = "basic_shapes";
  //   marker.id = 0;
  //   // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  //   marker.type = shape;
  //   // Set the marker action.  Options are ADD and DELETE
  //   marker.action = visualization_msgs::Marker::ADD;
  //   // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  //   marker.pose.position.x = 0;
  //   marker.pose.position.y = 0;
  //   marker.pose.position.z = 0;
  //   marker.pose.orientation.x = 0.0;
  //   marker.pose.orientation.y = 0.0;
  //   marker.pose.orientation.z = 0.0;
  //   marker.pose.orientation.w = 1.0;
  //   // Set the scale of the marker -- 1x1x1 here means 1m on a side
  //   marker.scale.x = 10;
  //   marker.scale.y = 10;
  //   marker.scale.z = 0.1;
  //   marker.lifetime = ros::Duration();
  //   marker_pub.publish(marker);
  // // }
  for (int h = 0; h < 21; h++)
  {
    for (int g = 0; g < 51; g++)
    {
  // for (int h = 0; h < 5; h++)
  // {
  //   for (int g = 0; g < 11; g++)
  //   {
  if (nh.hasParam("shelf_size") == false)
  {
    ROS_INFO("Set shelf_size");
    system("rosparam load `rospack find fcsc_description`/config/shelf_size.yaml");
      }
      private_nh.param<std::string>("shelf_name", shelf_name, "shelf");
      private_nh.param<std::string>("frame_id", shelf_pose.header.frame_id, "map");
      private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.01));
      private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.01));
      // private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.05));
      // private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.05));
      // private_nh.param<double>("x", shelf_pose.pose.position.x, 1.1);
      // private_nh.param<double>("y", shelf_pose.pose.position.y, 0);
      private_nh.param<double>("roll", roll, 0);
      private_nh.param<double>("pitch", pitch, 0);
      private_nh.param<double>("yaw", yaw, 0);
      shelf_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));

      object_visualizer.spawnShelf(shelf_name, shelf_pose);

      // int shelf = 0;
      
      for (int i = 0; i < 2; i++)
      {
        
        for (int j = 0; j < 2; j++)
        {

          for (int k = 0; k < 2; k++)
          {
            for (int m = 0; m < 1; m++)
            {
              count++;

              move_group.setNamedTarget("masita4");
              move_group.move();

              geometry_msgs::Pose target_pose1;

              // private_nh.param<std::string>("sandwich_name", sandwich_name, "sandwich");
              
              // private_nh.param<double>("roll", roll, 0);
              // private_nh.param<double>("pitch", pitch, 0);
              // private_nh.param<double>("yaw", yaw, 0);
              // if (shelf = 0){
              //   private_nh.param<std::string>("frame_id", sandwich_pose.header.frame_id, "shelf_board_1");
              //   private_nh.param<double>("x", sandwich_pose.pose.position.x, 0.05 + j*0.05);
              //   private_nh.param<double>("y", sandwich_pose.pose.position.y, 0.35+ k*0.2);
              //   private_nh.param<double>("z", sandwich_pose.pose.position.z, 0.11);
              //   ROS_INFO("shelf_board_1");
              // }else
              // {
              //   private_nh.param<std::string>("frame_id", sandwich_pose.header.frame_id, "shelf_board_2");
              //   private_nh.param<double>("x", sandwich_pose.pose.position.x, 0.06+j*0.1);
              //   private_nh.param<double>("y", sandwich_pose.pose.position.y, 0.4+k*0.15);
              //   private_nh.param<double>("z", sandwich_pose.pose.position.z, 0.054);
              //   ROS_INFO("shelf_board_2");
              // }
              
              // sandwich_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(deg2rad(roll), deg2rad(pitch), deg2rad(yaw));

              // moveit_visual_tools::FCSCVisualTools object_visualizer;

              // object_visualizer.spawnProduct(sandwich_name, sandwich_pose);
              // geometry_msgs::Pose target_pose2;

              // target_pose2.position.x = -1.0;   //max1.4min1.1
              // target_pose2.position.y = -0.45; //+-2.5
              // target_pose2.orientation.w = 1.0;
              // target_pose2.position.z = 0; //0.4,0.9,1.4
              // moveit::planning_interface::MoveGroupInterface::Plan plan2;

              // move_group2.setPoseTarget(target_pose2);
              // bool success_plan2 = (bool)move_group2.plan(plan2);

              // if (!success_plan2)
              // {
              //   ROS_INFO("daisha plan error1 count");
              // }
              // bool success_excute2 = (bool)move_group2.execute(plan2);

              // if (!success_excute2)
              // {
              //   ROS_INFO("daisha excute error1");
              // }
              // // 
              // target_pose2.position.x = 0;  //max1.4min1.1
              // target_pose2.position.y = 0;  //+-2.5
              // target_pose2.orientation.w = 1.0;
              // // target_pose2.position.z = 0; //0.4,0.9,1.4
              // move_group2.setPoseTarget(target_pose2);

              // move_group2.plan(plan2);
              // if (!success_plan2)
              // {
              //   ROS_INFO("plan error1 count");
              // }
              // move_group2.execute(plan2);

              // if (!success_excute2)
              // {
              //   ROS_INFO("excute error1");
              // }

              

              target_pose1.position.x = 1.0325 + 0.1; //max1.4min1.1
              target_pose1.position.y = -0.2 - 0.25; //+-2.5
              // target_pose1.position.x = 0.9;   //max1.4min1.1
              // target_pose1.position.y = -0.45; //+-2.5
              //target_pose1.position.z = 0.4;   //0.4,0.9,1.4
              target_pose1.position.z = 0.34; //0.4,0.9,1
              target_pose1.position.x -= (h * 0.01);
              target_pose1.position.y += (g * 0.01);
              // target_pose1.position.x -= (h * 0.05);
              // target_pose1.position.y += (g * 0.05);
              target_pose1.position.x += (0.15 * j);
              target_pose1.position.y += (0.4 * k);
              target_pose1.position.z += (0.545 * i);
              target_pose1.orientation.x = 0;
              target_pose1.orientation.y = 0.707107;
              target_pose1.orientation.z = 0;
              target_pose1.orientation.w = 0.707107;
              // target_pose1.orientation.x = 0;
              // target_pose1.orientation.z = 0;
              // target_pose1.orientation.w = 0;
              move_group.setPoseTarget(target_pose1);

              moveit::planning_interface::MoveGroupInterface::Plan plan;

              bool success_plan = (bool)move_group.plan(plan);

              if (!success_plan)
              {
                ROS_INFO("plan error1 count %d", (int)count);
               }else{
                 handler.saveTrajectoryToBag(plan, str);
               }


              bool success_execute = (bool)move_group.execute(plan);
              if (!success_execute)
              {
                ROS_INFO("execute error1");
                for (int l = 1; l <= 5; l++)
                {
                  move_group.setStartStateToCurrentState();
                  move_group.plan(plan);
                  success_execute = (bool)move_group.execute(plan);
                  if (!success_execute)
                  {
                    ROS_INFO("execute  error1 %d回目", (int)l + 1);
                    result_file << count << "," << shelf_pose.pose.position.x << "," << shelf_pose.pose.position.y << target_pose1.position.x << "," << target_pose1.position.y << "," << target_pose1.position.z << ","
                                << "error" << std::endl;
                    continue;
                  }
                  else
                  {
                    ROS_INFO("execute  success1 %d回目", (int)l + 1);
                    result_file << count << "," << shelf_pose.pose.position.x << "," << shelf_pose.pose.position.y << "," << target_pose1.position.x << "," << target_pose1.position.y << "," << target_pose1.position.z << ","
                                << "success" << std::endl;
                    break;
                  }
                }
              }
              else
              {

                if (!success_plan)
                {
                  // ROS_INFO("plan error1 count %d", (int)count);
                  result_file << count << "," << shelf_pose.pose.position.x << "," << shelf_pose.pose.position.y << "," << target_pose1.position.x << "," << target_pose1.position.y << "," << target_pose1.position.z << ","
                              << "error" << std::endl;
                  break;
                }
                else
                {
                  ROS_INFO("execute  success1 count %d", (int)count);
                  result_file << count << "," << shelf_pose.pose.position.x << "," << shelf_pose.pose.position.y << "," << target_pose1.position.x << "," << target_pose1.position.y << "," << target_pose1.position.z << ","
                              << "success" << std::endl;
                }
              }
              // object_visualizer.deleteObject(sandwich_name);
              // move_group3.move();
            }
          }
        }
        // shelf = 1;
      }
    }

    object_visualizer.deleteObject(shelf_name);
  }
  move_group.setNamedTarget("masita4");
  move_group.move();
  ros::shutdown();
  return 0;
}
