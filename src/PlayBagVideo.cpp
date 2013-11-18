/* 

   Andy McEvoy
   michael.mcevoy@colorado.edu

   24 - June - 2013

   Visual servoing to on a cubelet that is already in the camera's field of view

*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <iostream>
#include <baxter_msgs/ITB.h>
#include <cmath>

namespace enc = sensor_msgs::image_encodings;

bool pressed_state[2];

static const char WINDOW[] = "Original image";

class CubeletVisualServo {
  ros::NodeHandle nh_;
  ros::NodeHandle bh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  CubeletVisualServo(char* ros_image_stream): it_(nh_) {
    image_pub_ = it_.advertise("baxter_ar_tag_video", 1);
    image_sub_ = it_.subscribe(ros_image_stream, 1, &CubeletVisualServo::cubelet_VS, this);
    cv::namedWindow(WINDOW);
 }

  ~CubeletVisualServo() {
    cv::destroyWindow(WINDOW);
  }

  void cubelet_VS(const sensor_msgs::ImageConstPtr& msg)  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3); 
  }
};

int main(int argc, char** argv) {
  if (argc==2) {
    ros::init(argc, argv, "baxter_ar_tag_video");

    CubeletVisualServo cubeletVS(argv[1]);
    ros::spin();
    return 0;
  } else {
    std::cout<<"ERROR:\tusage - VisualServToARTAG <ros_image_topic>"<<std::endl;
    return 1;
  }
}
