#pragma once

#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<sensor_msgs/Image.h>
namespace depth_map{
    class DepthMap{
        public:
         DepthMap(ros::NodeHandle& nodeHandle);
         cv::Mat g1;
         cv::Mat g2;
         message_filters::Subscriber<sensor_msgs::Image> sub1;
         message_filters::Subscriber<sensor_msgs::Image> sub2;
         ros::NodeHandle nh_;
         virtual ~DepthMap();
        private:
        ros::Subscriber my_subscriber;
        ros::Subscriber my_subscriber2;
        void callback1(const sensor_msgs::ImageConstPtr& msg1);
        void callback2(const sensor_msgs::ImageConstPtr& msg1);
        void sync_callback();
        //  void sync_callback2(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2);
        //  ros::NodeHandle nh_;
        //  std::string scanTopicLeft_;
        //  std::string scanTopicRight_;
        //  cv::Mat g1,g2;
        //  message_filters::Subscriber<sensor_msgs::Image> sub1;
        //  message_filters::Subscriber<sensor_msgs::Image> sub2;
        //  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >sync;
        //  void sync_callback2(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2);
    };
}