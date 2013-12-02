#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <baxter_control/baxter_utilities.h>
#include <geometry_msgs/Pose.h>

// This code is a work in progress and would probably not compile
class VisualServo
{
private:
    ros::NodeHandle node_;
    ros::Subscriber pose_subscriber_;
    ros::Publisher target_publisher_;
    boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
    baxter_control::BaxterUtilities baxter_util_;
    std::string arm_;
    std::string planning_group_name_;
    std::string camera_frame_;
    std::vector<geometry_msgs::Pose> target_;
    tf::StampedTransform target_transform_;
public:
    VZ_Server() : arm_("left"),
		  planning_group_name_(arm_+"_arm"),
		  camera_frame_(arm+"camera")
	{
	    marker_pose_subscriber_ = node_.subscribe("ar_pose_marker",50, &VZ_Server::marker_pose_callback, this);
	    target_publisher_ = node_.advertise<geometry_msgs::Pose>("/visual_servo_target", 10);
	    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
	    move_group_->setPlanningTime(30.0);
	    // Let everything load
	    ros::Duration(1.0).sleep();
	    if( !baxter_util_.enableBaxter() )
		return;
	    servo_to_tag();
	    baxter_util_.disableBaxter();
	}

    bool servo_to_tag()
	{
	    while(ros::ok())
	    {
		moveit::planning_interface::MoveGroup::setPoseTargets(&target_, camera_frame_);
	    }
	}

    void marker_pose_callback(const geometry_msgs::Pose& target_marker)
    	{
	    // make sure we are looking at the same marker somewhere in here
	    // for whatever reason ar_track_alvar reports the codes position relative to the input frame without considering the rotation of the link containing the camera, we could fix that by setting up the ar node to report realtive to the arm and find the true transformation ourselves
	    tf::TransformListener::lookupTransform("base", input, ros::Time(0), target_transform_);
	    //add code to find a point a third of the way between the camera and the marker instead of the following
	    target_ = target_marker;
	    publisher_.publish(target_);
	}
}
    
int main(int argc, char **argv)
{
  ros::init (argc, argv, "ar_viz_servo");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  VisualServo visual_servo_object;
  ros::shutdown();
  return 0;
}


