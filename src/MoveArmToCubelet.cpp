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

int wheel_value=0;
int global_min_threshold=50;
int global_squareness_ratio=20;

bool pressed_state[2];

static const char WINDOW[] = "Original image";

void wheel_callback(const baxter_msgs::ITB::ConstPtr& msg) {
  int delta=0;
  if ((int)(msg->buttons[1])==1 && pressed_state[0]==0) { // top button pressed
    if (global_min_threshold>255) return;
    else global_min_threshold+=5;
  }
  if ((int)(msg->buttons[2])==1&& pressed_state[1]==0) { // bottom button pressed
    if (global_min_threshold<0) return;
    else global_min_threshold-=5;
  }
  pressed_state[0]=(int)(msg->buttons[1]);
  pressed_state[1]=(int)(msg->buttons[2]);
}

void update_global_min_threshold(int,void*) {
  //do nothing
}

void update_global_squareness_ratio(int,void*) {
  //do nothing
}

class CubeletVisualServo {
  ros::NodeHandle nh_;
  ros::NodeHandle bh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  CubeletVisualServo(char* ros_image_stream): it_(nh_) {
    image_pub_ = it_.advertise("correll_cubelet_visual_servo", 1);
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
    


    /* Gray scale image */
    cv::Mat filtered_image;
    cv::cvtColor(cv_ptr->image,filtered_image,CV_BGR2GRAY);
    
    // Image Filtering 
    cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
    cv::morphologyEx(filtered_image,filtered_image,cv::MORPH_CLOSE,element5);
    cv::morphologyEx(filtered_image,filtered_image,cv::MORPH_OPEN,element5);

    //cv::GaussianBlur(filtered_image,filtered_image,cv::Size(5,5),1.5);

    /*
    // Sharpen image
    cv::Mat sharpen_kernel(3,3,CV_32F,cv::Scalar(0));
    sharpen_kernel.at<float>(1,1)=5.0;
    sharpen_kernel.at<float>(0,1)=-1.0;
    sharpen_kernel.at<float>(2,1)=-1.0;
    sharpen_kernel.at<float>(1,0)=-1.0;
    sharpen_kernel.at<float>(1,2)=-1.0;
    cv::filter2D(filtered_image,filtered_image,filtered_image.depth(),sharpen_kernel);
    */

    cv::blur(filtered_image,filtered_image,cv::Size(3,3));    

    // Show final filtered image 
    cv::namedWindow("Filtered Image");
    cv::imshow("Filtered Image",filtered_image);

    // Finding Contours 
    cv::Mat contoured_image;
    cv::morphologyEx(filtered_image,contoured_image,cv::MORPH_GRADIENT,cv::Mat());
    cv::namedWindow("MORPH_GRADIENT contours");
    cv::imshow("MORPH_GRADIENT contours",contoured_image);

    // threshold image 
    cv::Mat threshold_image;
    cv::threshold(contoured_image,threshold_image,global_min_threshold,255,cv::THRESH_BINARY);
    cv::namedWindow("Threshold Image");
    cv::imshow("Threshold Image",threshold_image);
    cv::createTrackbar("Min Threshold:","Threshold Image",&global_min_threshold,255,update_global_min_threshold);
    update_global_min_threshold(global_min_threshold,0);
    
    // Make bounding boxes:
    //// detect edges using threshold image
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(threshold_image,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE);

    //// find the rotated rectangles
    std::vector<cv::RotatedRect> minRect(contours.size());
    cv::RotatedRect new_rectangle;
    
    double length_side_a, length_side_b;
    cv::Point2f new_rect_points[4];
    

    for (int i=0; i<contours.size(); i++) {
      // only keep track of rectangles that might be cubelets... needs work
      
      new_rectangle=cv::minAreaRect(cv::Mat(contours[i]));
      new_rectangle.points(new_rect_points);
      
      std::cout<<"INFO:\t point.x = "<<new_rect_points[0].x<<std::endl;
      length_side_a=pow(new_rect_points[0].x-new_rect_points[1].x,2)+
	pow(new_rect_points[0].y-new_rect_points[1].y,2);
      length_side_b=pow(new_rect_points[0].x-new_rect_points[3].x,2)+
	pow(new_rect_points[0].y-new_rect_points[3].y,2);
      
      std::cout<<"INFO:\tsize, l_a/l_b = "<<new_rectangle.size.area()<<" ,"<<length_side_a/length_side_b<<std::endl;
      if (new_rectangle.size.area()>=500 && // minimum size requirement
	  length_side_a/length_side_b>(100.0-global_squareness_ratio)/100.0 && // minimum squareness
	  length_side_a/length_side_b<(100.0+global_squareness_ratio)/100.0) {
	minRect[i]=new_rectangle;
      }
    }
    
    //// Draw contours & rectangles
    cv::Mat contour_output(threshold_image.size(),CV_8U,cv::Scalar(255));
    cv::Mat bounding_boxes=cv::Mat::zeros(contour_output.size(),CV_8UC3);;
    int i=0;
    for (; i >=0; i=hierarchy[i][0]) {
      cv::Point2f rect_points[4];
      minRect[i].points(rect_points);
      cv::drawContours(bounding_boxes,contours,i,cv::Scalar(255,255,255),1,8,hierarchy,0,cv::Point());
      for (int j=0; j<4; j++) {
      	cv::line(bounding_boxes,rect_points[j],rect_points[(j+1)%4],cv::Scalar(255),2,8);
      }
    }
    cv::namedWindow("bounding boxes");
    cv::imshow("bounding boxes",bounding_boxes);
    cv::createTrackbar("Out-of-square acceptance: ","bounding boxes",&global_squareness_ratio,100,update_global_squareness_ratio);
    update_global_squareness_ratio(global_squareness_ratio,0);

    /* Publish locations of Cubelets */
    cv_bridge::CvImage out_msg;
    out_msg.header=msg->header;
    out_msg.encoding=msg->encoding;
    out_msg.image=bounding_boxes;

    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
   
    image_pub_.publish(out_msg.toImageMsg());
  }
};

int main(int argc, char** argv) {
  if (argc==2) {
    ros::init(argc, argv, "correll_cubelet_visual_servo");

    // get value from right wheel
    ros::NodeHandle wheel_handle;
    ros::Subscriber wheel_sub=wheel_handle.subscribe("/sdk/robot/itb/right_itb/state",1,wheel_callback);

    CubeletVisualServo cubeletVS(argv[1]);
    ros::spin();
    return 0;
  } else {
    std::cout<<"ERROR:\tusage - VisualServoToCubelet <ros_image_topic>"<<std::endl;
    return 1;
  }
}
