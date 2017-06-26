#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include <iostream>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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


    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      // ######### 0529 add -start #########
      cv::Mat gray,prev,next;
      cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
      //cv_ptr->image = gray;
      //image_pub_.publish( cv_bridge::CvImage(std_msgs::Header(), "bgr8", gray).toImageMsg());
      // RGB..."bgr8" GRAY..."mono8"
      // ######### 0529 add -end #########

      if( gray.empty() ){
        cv::cvtColor( gray , prev , CV_BGR2GRAY ) ;
      }
      cv::cvtColor( gray , next , CV_BGR2GRAY ) ;

      std::vector<cv::Point2f> prev_pts ;
      std::vector<cv::Point2f> next_pts ;

      std::vector<cv::KeyPoint> keypoints;

      cv::GoodFeaturesToTrackDetector detector(300000, 0.1, 5);
      detector.detect( prev, keypoints);

      for( std::vector<cv::KeyPoint>::iterator itk = keypoints.begin(); itk != keypoints.end(); ++itk){
          prev_pts.push_back(itk->pt);
      }
      std::vector<uchar> status;
      std::vector<float> error;

      cv::calcOpticalFlowPyrLK(prev, next, prev_pts, next_pts, status, error, cv::Size(100,100), 3, cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.05), 0);

      std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
      std::vector<cv::Point2f>::const_iterator n = next_pts.begin();
      for(; n!=next_pts.end(); ++n,++p) {
          cv::line(gray, *p, *n, cv::Scalar(150,0,0),2);
      }


    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, gray);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg()); //COLOR
    image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(),"mono8",gray).toImageMsg() ); //GREY

    gray.copyTo(gray);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
