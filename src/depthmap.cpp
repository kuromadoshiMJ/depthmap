#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace cv;
using namespace message_filters;
using namespace std;
using namespace sensor_msgs;

Mat  g1, g2;
Mat disp;

int cnt1=0;
int cnt2=0;

void MatType( Mat inputMat ){
  int inttype = inputMat.type();

  string r, a;
  uchar depth = inttype & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (inttype >> CV_CN_SHIFT);
  switch ( depth ){
    case CV_8U:  r = "8U";   a = "Mat.at<uchar>(y,x)"; break;  
    case CV_8S:  r = "8S";   a = "Mat.at<schar>(y,x)"; break;  
    case CV_16U: r = "16U";  a = "Mat.at<ushort>(y,x)"; break; 
    case CV_16S: r = "16S";  a = "Mat.at<short>(y,x)"; break; 
    case CV_32S: r = "32S";  a = "Mat.at<int>(y,x)"; break; 
    case CV_32F: r = "32F";  a = "Mat.at<float>(y,x)"; break; 
    case CV_64F: r = "64F";  a = "Mat.at<double>(y,x)"; break; 
    default:     r = "User"; a = "Mat.at<UKNOWN>(y,x)"; break; 
  }   
  r += "C";
  r += (chans+'0');
  ROS_INFO_STREAM("Mat is of type " << r << " and should be accessed with " << a);
    
}
void sync_callback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2){

  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr cv_ptr2;

  try{ 
    cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8); 
  } 
  catch (cv_bridge::Exception& e){ 
    ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return; 
  }
  try{ 
    cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8); 
  } 
  catch (cv_bridge::Exception& e){ 
    ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return; 
  }

  cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);
  cv::cvtColor(cv_ptr2->image,g2,CV_BGR2GRAY);

  ROS_INFO_STREAM("Hello callback"<<cnt1<<cnt2);

  cv::Size s1= g1.size();
  cv::Size s2=g2.size();

  ROS_INFO("%d %d",s1.width,s2.width);

  if(s1.width==0)return;
  if(s2.width==0)return;
  
  cv::Mat disparity=cv::Mat(s1.width,s1.height,CV_8UC1);

  cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(); 

  stereo->setNumDisparities(80);
  stereo->setBlockSize(25);
  stereo->compute(g1,g2,disp);
  
  // minMaxLoc( disp, &minVal, &maxVal );
  // disp.convertTo(disparity, CV_8UC1, 255/(maxVal-minVal));

  normalize(disp, disparity, 0, 255, CV_MINMAX, CV_8U);

  ROS_INFO("Working fine");

  cv::imshow("left",g1);
  cv::imshow("right",g2);
  cv::imshow("disparity",disparity);

  cv::waitKey(100);
}
void sync_callback2(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2){
  ROS_INFO_STREAM("Hello callback222!!"<<cnt1<<cnt2);
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
  cv::cvtColor(cv_ptr2->image,g2,CV_BGR2GRAY);

  ROS_INFO_STREAM("Hello callback"<<cnt1<<cnt2);
  cv::Size s1= g1.size();
  cv::Size s2=g2.size();
  ROS_INFO("%d %d",s1.width,s2.width);

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

void sync_callback2(){
  ROS_INFO_STREAM("Hello callback222!!"<<cnt1<<cnt2);

  // cv_bridge::CvImagePtr cv_ptr;
  // cv_bridge::CvImagePtr cv_ptr2;

  // try{ 
  //   cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8); 
  // } 
  // catch (cv_bridge::Exception& e){ 
  //   ROS_ERROR("cv_bridge exception: %s", e.what()); 
  //   return; 
  // }
  // try{ 
  //   cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8); 
  // } 
  // catch (cv_bridge::Exception& e){ 
  //   ROS_ERROR("cv_bridge exception: %s", e.what()); 
  //   return; 
  // }
  // cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);
  // cv::cvtColor(cv_ptr2->image,g2,CV_BGR2GRAY);
 
  cv::Size s1= g1.size();
  cv::Size s2=g2.size();
  
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

  normalize(disp, disparity, 0, 255, CV_MINMAX, CV_8U);
  ROS_INFO("Working fine");

  cv::imshow("left",g1);
  cv::imshow("right",g2);
  cv::imshow("disparity",disparity);

  cv::waitKey(100);

}

void sync_callback3(){
  ROS_INFO_STREAM("Hello callback222!!"<<cnt1<<cnt2);
  
  // Mat img1=imread("/home/kuromadoshi/ws2/src/depthmap/src/ambush_5_left.jpg");
  // Mat img2=imread("/home/kuromadoshi/ws2/src/depthmap/src/ambush_5_right.jpg");
  Mat img1=imread("/home/kuromadoshi/Downloads/tsubuka_l.png");
  Mat img2=imread("/home/kuromadoshi/Downloads/tsubuka_r.png");
  cv::cvtColor(img1,g1,CV_BGR2GRAY);
  cv::cvtColor(img2,g2,CV_BGR2GRAY);
  cv::Size s1= g1.size();
  cv::Size s2=g2.size();

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
  double maxVal;
  double minVal;
  minMaxLoc( disp, &minVal, &maxVal );
  disp.convertTo(disparity, CV_8UC1, 255/(maxVal-minVal));

  ROS_INFO("Working fine");

  cv::imshow("left",g1);
  cv::imshow("right",g2);
  cv::imshow("disparity",disparity);

  cv::waitKey(0);

}
void callback1(const sensor_msgs::ImageConstPtr& msg1){
  cnt1++;
  ROS_INFO("HELLO");
  cv_bridge::CvImagePtr cv_ptr; 
  try{ 
    cv_ptr = cv_bridge::toCvCopy(msg1, sensor_msgs::image_encodings::BGR8); 
  } 
  catch(cv_bridge::Exception& e){ 
    ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return; 
  }

  cv::cvtColor(cv_ptr->image,g1,CV_BGR2GRAY);

}
void callback2(const sensor_msgs::ImageConstPtr& msg2){
  cnt2++;
  cv_bridge::CvImagePtr cv_ptr2; 
  try{ 
    cv_ptr2 = cv_bridge::toCvCopy(msg2, sensor_msgs::image_encodings::BGR8); 
  } 
  catch(cv_bridge::Exception& e){ 
    ROS_ERROR("cv_bridge exception: %s", e.what()); 
    return; 
  } 
  cv::cvtColor(cv_ptr2->image,g2,CV_BGR2GRAY);
  ROS_INFO("Entering sync_callback");
  sync_callback2();

}







int main(int argc, char **argv){
  ros::init(argc, argv, "depthmap");
  ros::NodeHandle nh;
  
  // ros::Subscriber sub1=nh.subscribe("/cam0/image_raw",1,callback1);
  // ros::Subscriber sub2=nh.subscribe("/cam1/image_raw",1,callback2);
  // ros::Subscriber sub1=nh.subscribe("/image_topic_1",10,callback1);
  // ros::Subscriber sub2=nh.subscribe("/image_topic_2",10,callback2);
  ROS_INFO("Callback entered");
  sync_callback3();
  
  // message_filters::Subscriber<sensor_msgs::Image> sub1(nh,"/cam0/image_raw",1);
  // message_filters::Subscriber<sensor_msgs::Image> sub2(nh,"/cam1/image_raw",1);
  // TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >sync(sub1,sub2,1);
  
  // ROS_INFO("Callback entered");
  // sync.registerCallback(boost::bind(sync_callback2, _1, _2));
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}