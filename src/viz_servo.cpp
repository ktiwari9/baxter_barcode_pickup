#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class VisualServo
{
private:
  ros::NodeHandle node_;
  image_transport::ImageTransport transporter_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
  std::string arm_;
  std::string planning_group_name_;
  std::string camera_image_;
  tf::StampedTransform target_;
  //std::vector<geometry_msgs::Pose> target_;
public:
  VisualServo() : transporter_(node_),  
		  arm_("left"),
		  planning_group_name_(arm_+"_arm"),
		  camera_image_("/cameras/"+arm_+"_hand_camera/image")
  {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(20.0);
    move_group_->setPlannerId("RRTstarkConfigDefault");
    image_pub_ = transporter_.advertise("image_tracker", 1);
    image_sub_ = transporter_.subscribe(camera_image_, 1, &VisualServo::tracker, this);
    cv::namedWindow("Baxter Image");
    cv::namedWindow("Threshold Image");
    while(ros::ok())
      {
	servo_to_tag();
      }
  }

  bool servo_to_tag()
  	{
	      move_group_->setRandomTarget();
	      move_group_->move();
// bool 	setPositionTarget (double x, double y, double z, Aconst std::string &end_effector_link="")
//  	Set the goal position of the end-effector end_effector_link to be (x, y, z). If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setRPYTarget (double roll, double pitch, double yaw, const std::string &end_effector_link="")
//  	Set the goal orientation of the end-effector end_effector_link to be (roll,pitch,yaw) radians. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setOrientationTarget (double x, double y, double z, double w, const std::string &end_effector_link="")
//  	Set the goal orientation of the end-effector end_effector_link to be the quaternion (x,y,z,w). If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTarget (const Eigen::Affine3d &end_effector_pose, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTarget (const geometry_msgs::Pose &target, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTarget (const geometry_msgs::PoseStamped &target, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTargets (const EigenSTL::vector_Affine3d &end_effector_pose, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. In this case the goal pose can be any of the ones specified in the array. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTargets (const std::vector< geometry_msgs::Pose > &target, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. In this case the goal pose can be any of the ones specified in the array. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// bool 	setPoseTargets (const std::vector< geometry_msgs::PoseStamped > &target, const std::string &end_effector_link="")
//  	Set the goal pose of the end-effector end_effector_link. In this case the goal pose can be any of the ones specified in the array. If end_effector_link is empty (the default value) then the end-effector reported by getEndEffectorLink() is assumed.
 
// void 	setPoseReferenceFrame (const std::string &pose_reference_frame)
//  	Specify which reference frame to assume for poses specified without a reference frame.
 
// bool 	setEndEffectorLink (const std::string &link_name)
//  	Specify the link the end-effector to be considered is attached to.
 
// bool 	setEndEffector (const std::string &eef_name)
//  	Specify the name of the end-effector to use.
 
// void 	clearPoseTarget (const std::string &end_effector_link="")
//  	Forget pose specified for the end-effector end_effector_link.
 
// void 	clearPoseTargets ()
//  	Forget any poses specified for any end-effector. 
  	}

  void tracker(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
  	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
  	ROS_ERROR("cv_bridge exception: %s", e.what());
  	return;
      }

    // threshold image in green range for tracking
    // values taken for light and dark areas on the cube face using GIMP
    cv::Mat threshold_image;
    cv::inRange(cv_ptr->image,cv::Scalar(60,60,180),cv::Scalar(120,120,255),threshold_image);
    cv::imshow("Threshold Image",threshold_image);
        
    // find the central moments of the image, and divide by area to get the centroid
    cv::Moments image_moment;
    image_moment = moments(threshold_image);
    double x = image_moment.m10/image_moment.m00;
    double y = image_moment.m01/image_moment.m00;

    cv::imshow("Baxter Image", cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
  }


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


