#include <ros/ros.h>
//http://wiki.ros.org/rosbag/Code%20API
//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
///opt/ros/hydro/include/opencv2/highgui/highgui_c.h 
int main(int argc, char ** argv)
{
    if (argc != 3)
    {
	ROS_INFO("usage: video_to_bag <bag_file> <video_file>");
	return 1;
    }    
    rosbag::Bag bag;
    bag.open(argv[1], rosbag::bagmode::Write);
   
    CvCapture* capture = cvCreateFileCapture(argv[2]);
    if(!capture)
    {
	ROS_ERROR("Could not open video file: %s", argv[2]);
	exit(0);  
    }
  
    IplImage* img = 0; 
    while(cvGrabFrame(capture))
    {
	
	img=cvRetrieveFrame(capture);           // retrieve the captured frame
    
	sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(img, "bgr8");
    
	bag.write("image", time, msg);
	time = time + ros::Duration((double) 1/fps); // get the time in the bag file correct
    }
    cvReleaseCapture(&capture);
    bag.close();
}
