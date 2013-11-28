#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <baxter_control/baxter_utilities.h>
#include <baxter_pick_place/custom_environment2.h>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "ar_viz_serv"o);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::shutdown();

  return 0;
}


