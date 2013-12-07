#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

class VisualServo
{
private:
  ros::NodeHandle node_;
  // ros::Subscriber marker_pose_subscriber_;
  // ros::Publisher target_publisher_;
    
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
  std::string arm_;
  std::string planning_group_name_;
  std::string camera_frame_;
  //std::vector<geometry_msgs::Pose> target_;
  tf::StampedTransform target_;
public:
  VisualServo() : arm_("left"),
		planning_group_name_(arm_+"_arm"),
		camera_frame_(arm_+"_hand_camera")
  {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(30.0);
    move_group_->setPlannerId("RRTstark");
    while(ros::ok())
      {
	// marker_pose_subscriber_ = node_.subscribe("ar_pose_marker",50, &VisualServo::marker_pose_callback, this);
	// target_publisher_ = node_.advertise<geometry_msgs::Pose>("/visual_servo_target", 10);
	servo_to_tag();
      }
  }

  bool servo_to_tag()
  	{
	      move_group_->setRandomTarget();
	      move_group_->move();
	      //moveit::planning_interface::MoveGroup::setPoseTargets(&target_, camera_frame_);
  	}

  // void marker_pose_callback(const geometry_msgs::Pose& target_marker)
  // {
  //   //tf::TransformListener::lookupTransform("base", input, ros::Time(0), target_transform_);
  //   //target_ = target_marker;
  //   //publisher_.publish(target_);
  // }
};
    
  int main(int argc, char **argv)
  {
    ros::init (argc, argv, "ar_viz_servo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    VisualServo vizual_servo;
    ros::shutdown();
    return 0;
  }


