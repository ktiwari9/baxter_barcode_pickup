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
//#include <baxter_gripper_server/electric_parallel_gripper.h>

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
  double x_correct,y_correct,z_height;
  double step_size, error_thresh;
  double pixel_num;
  geometry_msgs::PoseStamped pose;
  bool reset;
  //  baxter_gripper_server::ElectricParallelGripper left_gripper;
public:
  VisualServo() : transporter_(node_),  
		  arm_("left"),
		  planning_group_name_(arm_+"_arm"),
		  hand_camera_(arm_+"_hand_camera"),
		  camera_image_("/cameras/"+hand_camera_+"/image")
		  //		  left_gripper("baxter_left_gripper_action/gripper_action","left", false)
		  
  {
    move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
    move_group_->setPlanningTime(20.0);
    move_group_->setPlannerId("KPIECEkConfigDefault");
    image_pub_ = transporter_.advertise("image_tracker", 1);
    image_sub_ = transporter_.subscribe(camera_image_, 1, &VisualServo::tracker, this);
    cv::namedWindow("Baxter Image");
    cv::namedWindow("Threshold Image");
    pose.header.frame_id = "base";  	
    pose.pose.position.x = 0.4;
    pose.pose.position.y = 0.2;
    pose.pose.position.z = 0.35;
    pose.pose.orientation.x = 1;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 0.0274;
    move_group_->setPoseTarget(pose,"left_wrist");
    move_group_->move();
    move_group_->move();
    z_height = 0.4;

    // ROS_INFO_STREAM("WTF???\N");
    // left_gripper.openGripper();
    // ros::Duration(2.0).sleep();
    // left_gripper.closeGripper();
    // ros::Duration(2.0).sleep();
    // left_gripper.openGripper();
    // ros::Duration(2.0).sleep();
    // left_gripper.closeGripper();
    // ROS_INFO_STREAM("HUMM???\N");
    while(ros::ok())
      {
	servo_to_tag();
      }
  }


  bool servo_to_tag()
  	{

	  step_size = 0.05 ;
	  error_thresh = 0.1 - 0.08*z_height/.4;
	  if (error_x_ > error_thresh)
	    {
	      if (error_y_ > error_thresh)
	  	{
	  	  y_correct = -step_size;
	  	  x_correct = -step_size;
	  	}
	      else if ( error_y_ < -error_thresh) 
	  	{
	  	  y_correct = step_size;
	  	  x_correct = -step_size;
	  	}
	    }
	  else if ( error_x_ < -error_thresh) 
	    {
	      if (error_y_ > error_thresh)
	  	{
	  	  y_correct = -step_size;
	  	  x_correct = step_size;
	  	}
	      else if ( error_y_ < -error_thresh) 
	  	{
	  	  y_correct = step_size;
	  	  x_correct = step_size;
	  	}
	    }
	  else
	    {
	      x_correct = 0;
	      y_correct = 0;
	      if (z_height > 0.02)
	  	z_height = z_height - .05;
	      else
		{
		  z_height = -.05;
		}

	    }
	  pose.pose.position.x = tf_transform_.getOrigin().x() + y_correct;
	  pose.pose.position.y = tf_transform_.getOrigin().y() + x_correct;
	  pose.pose.position.z = z_height;  
	  pose.pose.orientation.x = 1;
	  pose.pose.orientation.y = 0;
	  pose.pose.orientation.z = 0;
	  pose.pose.orientation.w = 0.0274;
	  move_group_->setPoseTarget(pose,"left_wrist");
	  move_group_->move();
	  tf_listener_.lookupTransform("/base","/left_hand_camera", ros::Time(0), tf_transform_);

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
    pixel_num = image_moment.m00;

    // if (image_moment.m00 < 20)
    //   {
    //   }
    // else
    //   {
    // 	error_y_ = (height_center-y)/height_center;
    // 	error_x_ = (width_center-x)/width_center;
    //   }
    ROS_INFO("%f,%f,%f,%f\n",x,y,error_x_,error_y_);
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


