#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<opencv2/opencv.hpp>
#include "opencv2/ximgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace cv;
using namespace message_filters;
using namespace std;
using namespace sensor_msgs;
using namespace cv::ximgproc;

Mat lft;
Mat rgt;
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

void sync_callback3(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2){
  ROS_INFO_STREAM("Hello callback222!!"<<cnt1<<cnt2);
  
  Mat img1=imread("/home/kuromadoshi/ws2/src/depthmap/src/ambush_5_left.jpg");
  Mat img2=imread("/home/kuromadoshi/ws2/src/depthmap/src/ambush_5_right.jpg");
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
  cv::cvtColor(cv_ptr->image,lft,CV_BGR2GRAY);

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
  cv::cvtColor(cv_ptr2->image,rgt,CV_BGR2GRAY);
  // Mat img1=imread("/home/kuromadoshi/Downloads/ambush_5_right.jpg");
  // Mat img2=imread("/home/kuromadoshi/Downloads/ambush_5_left.jpg");
  // cv::cvtColor(img1,lft,CV_BGR2GRAY);
  // cv::cvtColor(img2,rgt,CV_BGR2GRAY);
  // cv::imshow("img1",rgt);
  // cv::waitKey(0);
  // return;
  cv::Size s1= lft.size();
  cv::Size s2=rgt.size();

   Mat left_for_matcher, right_for_matcher;
   Mat left_disp,right_disp;
   Mat left_disp_resized;
   Mat filtered_disp,solved_disp,solved_filtered_disp;
   Mat conf_map = Mat(lft.rows,lft.cols,CV_8U);
   conf_map = Scalar(255);
   Rect ROI;
   Ptr<DisparityWLSFilter> wls_filter;

    resize(lft ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
    resize(rgt,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
  
   int max_disp=160;
   int wsize=5;
  // cv::Mat disparity=cv::Mat(s1.width,s1.height,CV_8UC1);
  
  
  int preFilterCap = 63;
  double fbs_lambda = 128.0; // donno what it does
  double vis_mult =3.0; // set//changes brightness of pixel
  double fbs_luma=16.0; // smoothens out the image
  double fbs_chroma=16.0; //makes it sharp
  double fbs_spatial=8.0; //some kind of threshold higher value ma
  // cv::Ptr<cv::StereoSGBM> left_matcher= cv::StereoSGBM::create(16,max_disp,wsize)   ; 

  
  // left_matcher->setPreFilterCap(preFilterCap);
  // left_matcher->setP1(24* (wsize) * (wsize));
  // left_matcher->setP2(96 * (wsize) * (wsize));
  
  Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
  wls_filter = createDisparityWLSFilter(left_matcher);

  Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
  left_matcher->compute(left_for_matcher,right_for_matcher, left_disp);  
  right_matcher->compute(right_for_matcher,left_for_matcher,right_disp);
  //  cv::imshow("img1",left_disp);
  //  cv::imshow("img2",right_disp);
  // cv::waitKey(0);
  // return;
  double lambda=8000;
  double sigma=1.5;
   wls_filter->setLambda(lambda);
   wls_filter->setSigmaColor(sigma);
   MatType(left_disp);
   MatType(right_disp);
   ROS_INFO_STREAM("fuck"<<left_disp.depth()<<"fuck"<<CV_16S<<"fuck"<<!left_disp.empty());
   ROS_INFO_STREAM("fuck"<<lft.depth()<<"fuck"<<CV_8U<<"fuck"<<!lft.empty());
   ROS_INFO_STREAM("ans"<<(!left_disp.empty() && (left_disp.depth() == CV_8U || left_disp.depth() == CV_16S || left_disp.depth() == CV_16U || left_disp.depth() == CV_32F) && left_disp.channels()<=4));
   wls_filter->filter(left_disp,lft,filtered_disp,right_disp,Rect(),rgt);
   ROS_INFO_STREAM("hello");
   conf_map = wls_filter->getConfidenceMap();
    // Mat left_disp_resized;
    resize(left_disp,left_disp_resized,lft.size());
    ROI = wls_filter->getROI();
        
            // upscale raw disparity and ROI back for a proper comparison:
            resize(left_disp,left_disp,Size(),2.0,2.0);
            left_disp = left_disp*2.0;
            left_disp_resized = left_disp_resized*2.0;
            ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        
   fastBilateralSolverFilter(lft, left_disp_resized, conf_map/255.0f, solved_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
  	ROS_INFO_STREAM("hello");
    fastBilateralSolverFilter(lft, filtered_disp, conf_map/255.0f, solved_filtered_disp, fbs_spatial, fbs_luma, fbs_chroma, fbs_lambda);
   Mat raw_disp_vis;
   getDisparityVis(left_disp,raw_disp_vis,vis_mult);
   namedWindow("raw disparity", WINDOW_AUTOSIZE);
   imshow("raw disparity", raw_disp_vis);
   Mat filtered_disp_vis;
   getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
   namedWindow("filtered disparity", WINDOW_AUTOSIZE);
   imshow("filtered disparity", filtered_disp_vis);

    if(!solved_disp.empty()){
        Mat solved_disp_vis;
        getDisparityVis(solved_disp,solved_disp_vis,vis_mult);
        namedWindow("solved disparity", WINDOW_AUTOSIZE);
        imshow("solved disparity", solved_disp_vis);

        Mat solved_filtered_disp_vis;
        getDisparityVis(solved_filtered_disp,solved_filtered_disp_vis,vis_mult);
        namedWindow("solved wls disparity", WINDOW_AUTOSIZE);
        imshow("solved wls disparity", solved_filtered_disp_vis);
    }

        

  // minMaxLoc( disp, &minVal, &maxVal );
  // disp.convertTo(disparity, CV_8UC1, 255/(maxVal-minVal));

  ROS_INFO("Working fine");

  // cv::imshow("lft",g1);
  // cv::imshow("rgt",g2);
  // cv::imshow("disparity",disparity);
  cv::imshow("img1",lft);
  cv::waitKey(500);

}

int main(int argc, char **argv){
  ros::init(argc, argv, "filtered_disparity");
  ros::NodeHandle nh;
  

  
  // ROS_INFO("Callback entered");
  // sync_callback3();
  message_filters::Subscriber<sensor_msgs::Image> sub1(nh,"/cam0/image_raw",1);
  message_filters::Subscriber<sensor_msgs::Image> sub2(nh,"/cam1/image_raw",1);
  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image >sync(sub1,sub2,1);
  
  ROS_INFO("Callback entered");
  sync.registerCallback(boost::bind(sync_callback3, _1, _2));
  
  ros::spin();
  cv::destroyAllWindows();
  return 0;
}