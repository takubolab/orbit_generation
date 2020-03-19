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
#include <sys/stat.h>
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
  std::string str("trajectory"); // 保存ファイル名の変更
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
  std::stringstream directory_path;
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

  directory_path << ros::package::getPath("orbit_generation") + "/bag/" + str;
  mkdir(directory_path.str().c_str(), 0775);

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  for (int h = 0; h < 21; h++) //棚の奥行きの移動回数
  {
    for (int g = 0; g < 51; g++) // 棚の左右の移動回数
    {
      private_nh.param<std::string>("shelf_name", shelf_name, "shelf");
      private_nh.param<std::string>("frame_id", shelf_pose.header.frame_id, "map");
      private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.01)); //基本的には1.1 ~ 1.3の間を移動させる。初期姿勢によるが最低値は1.0より小さいと貫通する可能性がある
      private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.01));     //基本的に棚の左右は-0.25 ~ 0.25の間を移動させるがそれより大きくしても問題はない
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
            count++;

            move_group.setNamedTarget("masita4"); //初期姿勢
            move_group.move();

            geometry_msgs::Pose target_pose1;
            //目標位置の開始位置の決定
            target_pose1.position.x = 1.0325 + 0.1;
            target_pose1.position.y = -0.2 - 0.25;
            target_pose1.position.z = 0.34;
            // 棚の位置変更の補正
            target_pose1.position.x -= (h * 0.01);
            target_pose1.position.y += (g * 0.01);
            //目標位置のずらす間隔の設定
            target_pose1.position.x += (0.15 * i);
            target_pose1.position.y += (0.4 * j);
            target_pose1.position.z += (0.545 * k);
            //手先の角度
            target_pose1.orientation.x = 0;
            target_pose1.orientation.y = 0.707107;
            target_pose1.orientation.z = 0;
            target_pose1.orientation.w = 0.707107;
            move_group.setPoseTarget(target_pose1);

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            bool success_plan = (bool)move_group.plan(plan);

            if (!success_plan)
            {
              ROS_INFO("plan error1 count %d", (int)count);
            }
            else
            {
              handler.saveTrajectoryToBag(plan, directory_path.str());
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
              }
              else
              {
                ROS_INFO("execute  success1 count %d", (int)count);
                result_file << count << "," << shelf_pose.pose.position.x << "," << shelf_pose.pose.position.y << "," << target_pose1.position.x << "," << target_pose1.position.y << "," << target_pose1.position.z << ","
                            << "success" << std::endl;
              }
            }
          }
        }
        move_group.setNamedTarget("masita4");
        move_group.move();
      }
    }
    object_visualizer.deleteObject(shelf_name);
  }
  move_group.setNamedTarget("masita4");
  move_group.move();
  ros::shutdown();
  return 0;
}
