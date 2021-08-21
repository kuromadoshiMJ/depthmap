#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
// #include "opencv2/contrib/contrib.hpp"
using namespace cv;
using namespace message_filters;
using namespace std;


void Callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2){
    
    // Mat left ;
    // Mat right;
    Mat  g1, g2;
    
    cv_bridge::CvImagePtr cv_ptr; 
    try 
    { 
      cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8); 
    } 
    catch (cv_bridge::Exception& e) 
    { 
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
      return; 
    }
    cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);
    
    cv_bridge::CvImagePtr cv_ptr2; 
    try 
    { 
      cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8); 
    } 
    catch (cv_bridge::Exception& e) 
    { 
      ROS_ERROR("cv_bridge exception: %s", e.what()); 
      return; 
    } 
    cv::cvtColor(cv_ptr->image,g2,CV_BGR2GRAY);
    
    

// int numDisparities = 16;
// int blockSize = 5;
int preFilterType = 1;
int preFilterSize = 1;
int preFilterCap = 31;
int minDisparity = 0;
int textureThreshold = 10;
int uniquenessRatio = 15;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;

cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create 	( ) 	;	
cv::Mat disp, disparity;
stereo->setNumDisparities(16);
stereo->setBlockSize(15);
stereo->compute(g1,g2,disp);

cv::imshow("disparity",disp);
// streo->setNumDisparities(numDisparities);
// streo->setBlockSize
cv::waitKey(0);
// stereo->compute(Left_nice,Right_nice,disp);


}
int main(int argc, char **argv){
    ros::init(argc, argv, "depthmap");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> sub1(nh,"/camera_info_topic_1",10);
    message_filters::Subscriber<sensor_msgs::Image> sub2(nh,"/camera_info_topic_2",10);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >sync(sub2,sub1,10);
    sync.registerCallback(boost::bind(&Callback, _1, _2));

    ros::spin();
    return 0;
}