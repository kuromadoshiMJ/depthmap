#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "depthmap/DepthMap.hpp"
// #define r1 DepthMap::r1;
// #define g2 DepthMap::r2;
void depth_map::DepthMap::sync_callback(){
        ROS_INFO_STREAM("CallBack Entered");
        // using namespace depth_map;
        
        // cv::Mat g1=DepthMap::r1;
        // cv::Mat g2=DepthMap::r2;
        cv::Mat disp;
        // cv_bridge::CvImagePtr cv_ptr;
        // try 
        // { 
        //     cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8); 
        // } 
        // catch (cv_bridge::Exception& e) 
        // { 
        //     ROS_ERROR("cv_bridge exception: %s", e.what()); 
        //     return; 
        // }
        // cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);

        // cv_bridge::CvImagePtr cv_ptr2;
        // try 
        // { 
        //     cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8); 
        // } 
        // catch (cv_bridge::Exception& e) 
        // { 
        //     ROS_ERROR("cv_bridge exception: %s", e.what()); 
        //     return; 
        // }
        // cv::cvtColor(cv_ptr2->image,g2,CV_BGR2GRAY);

        // ROS_INFO_STREAM("Hello callback"<<cnt1<<cnt2);
        cv::Size s1= g1.size();
        cv::Size s2=g2.size();
        ROS_INFO("%d %d",s1.width,s2.width);
        if(s1!=s2)return;

        cv::Mat disparity=cv::Mat(s1.width,s1.height,CV_8UC1);
        int numDisparities = 5;
        int blockSize = 1;
        int preFilterType = 1;
        int preFilterSize = 1;
        int preFilterCap = 15;
        int minDisparity = 0;
        int textureThreshold = 10;
        int uniquenessRatio = 5;
        int speckleRange = 2;
        int speckleWindowSize = 150;
        int disp12MaxDiff = -1;
        int dispType = CV_16S;
        // double minVal; double maxVal;

        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create  ()   ; 

        sgbm->setMinDisparity(minDisparity);
        sgbm->setPreFilterCap(preFilterCap);
        sgbm->setP1(8 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setP2(32 * (2 * blockSize + 1) * (2 * blockSize + 1));
        sgbm->setBlockSize((2 * blockSize + 1));
        sgbm->setNumDisparities(16 * numDisparities);
        sgbm->setUniquenessRatio(uniquenessRatio);
        sgbm->setSpeckleWindowSize(speckleWindowSize);
        sgbm->setSpeckleRange(speckleRange);
        sgbm->setDisp12MaxDiff(disp12MaxDiff);
        sgbm->compute(g1, g2, disp);  

        // minMaxLoc( disp, &minVal, &maxVal );
        // disp.convertTo(disparity, CV_8UC1, 255/(maxVal-minVal));
        normalize(disp, disparity, 0, 255, CV_MINMAX, CV_8U);
        
        cv::imshow("left",g1);
        cv::imshow("right",g2);
        cv::imshow("disparity",disparity);

        cv::waitKey(100);
    }
namespace depth_map{
    DepthMap::DepthMap(ros::NodeHandle &nodeHandle):nh_(nodeHandle){
        ROS_INFO_STREAM("Object Created");

        //sub1.subscribe(nh_,"/cam0/image_raw", 1);
        //sub2.subscribe(nh_,"/cam1/image_raw",1);
        // sync=message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >(sub1,sub2,1);
        // ROS_INFO_STREAM("Reached cllback");
        // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >sync(sub1,sub2,1);
        // sync.registerCallback(&DepthMap::sync_callback2,this);
        // sync.registerCallback(boost::bind(sync_callback2, _1, _2));
        my_subscriber=nh_.subscribe("/cam0/image_raw",1,&DepthMap::callback1,this);
        my_subscriber2=nh_.subscribe("/cam1/image_raw",1,&DepthMap::callback2,this);
        ROS_INFO_STREAM("Reached callback");
    }
    DepthMap::~DepthMap(){

    }
    void DepthMap::callback1(const sensor_msgs::ImageConstPtr& msg){
        ROS_INFO_STREAM("Fuck");
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
        cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);
    }
    void DepthMap::callback2(const sensor_msgs::ImageConstPtr& msg){
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
        cv::cvtColor(cv_ptr->image,g2,CV_BGR2GRAY);
        sync_callback();

    }
    
}
