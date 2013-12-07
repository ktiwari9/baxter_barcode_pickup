#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <iostream>

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
  std::string hand_camera_;
  std::string camera_image_;
  tf::StampedTransform target_;
  double error_x_;
  double error_y_;
  std::vector<double> last_rpy_;
  std::vector<double> next_rpy_;
  tf::TransformListener tf_listener_;
  tf::StampedTransform tf_transform_;
  // planning_interface::MotionPlanRequest req;
  // planning_interface::MotionPlanResponse res;
  // planning_interface::PlannerManagerPtr planner_instance;
public:
  VisualServo() : transporter_(node_),  
		  arm_("left"),
		  planning_group_name_(arm_+"_arm"),
		  hand_camera_(arm_+"_hand_camera"),
		  camera_image_("/cameras/"+hand_camera_+"/image")
		  
  {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(10.0);
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
	  tf_listener_.lookupTransform("/base","/left_hand_camera", ros::Time(0), tf_transform_);
	  geometry_msgs::PoseStamped pose;
	  std::cout << error_x_ << "," << error_y_ << "\n";
	  std::cout << tf_transform_.getOrigin().x() << "," << tf_transform_.getOrigin().y() << "," << tf_transform_.getOrigin().z()  << "\n";
	  if (error_x_ < 600 || error_x_ > 5 || error_y_ < 600 || error_x_ > 5)
	    {
	      pose.header.frame_id = "base";  	
	      pose.pose.position.x = tf_transform_.getOrigin().x()+error_x_*.0001;
	      pose.pose.position.y = tf_transform_.getOrigin().y()+error_y_*.0001;
	      pose.pose.position.z = tf_transform_.getOrigin().z();  
	      pose.pose.orientation.x = tf_transform_.getRotation().x();
	      pose.pose.orientation.y = tf_transform_.getRotation().y();
	      pose.pose.orientation.z = tf_transform_.getRotation().z();
	      pose.pose.orientation.w = tf_transform_.getRotation().w();
	      std::cout << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << "\n";
	      move_group_->setPoseTarget(pose,"left_wrist");
	      move_group_->move();
	    }
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
    double height_center = cv_ptr->image.rows/2;
    double width_center = cv_ptr->image.cols/2;
    error_y_ = (height_center-y);
    error_x_ = (width_center-x);
    //ROS_INFO("%f,%f,%f,%f\n",x,y,error_x_,error_y_);
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


