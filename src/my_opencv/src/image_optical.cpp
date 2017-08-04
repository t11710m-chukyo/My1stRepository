#include <ros/ros.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "iostream"
#include "math.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/Labeling.h"

#define MAX_VAL 0
#define MIN_VAL 255
#define FG      255
#define BG      0

using namespace std;
using namespace cv;
int frame_count = 0;

cv::Mat image; 
cv::Mat prev_image; 

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
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
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

    cv::cvtColor( cv_ptr->image, image, CV_BGR2GRAY );
    image_pub_.publish( cv_bridge::CvImage( std_msgs::Header(), "bgr8", image).toImageMsg());
    
    // Setting the image for display
    Mat org_flow = cv::Mat::ones(cv::Size(image.cols,image.rows),CV_8UC1)*BG;

    if( frame_count == 0 ) prev_image = image.clone();

    if( frame_count > 0 )
    {
    	vector<cv::Point2f> prev_pts;
    	vector<cv::Point2f> next_pts;
    	
      // Define of the number of optical flow (To detect a object:image size, To display a flow:one-tens image size) 
    	Size flowSize(image.rows,image.cols);
    	
    	Point2f center = cv::Point(image.cols/2., image.rows/2.);
      for(int i=0; i<flowSize.width; ++i)
      {
      	for(int j=0; j<flowSize.width; ++j)
      	{
      		Point2f p(i*float(image.cols)/(flowSize.width-1), j*float(image.rows)/(flowSize.height-1));
          prev_pts.push_back((p-center)*0.95f+center);
        }
      }
    
      // Calculation of optical flow
    	Mat f;
    	calcOpticalFlowFarneback(prev_image, image, f, 0.8, 10, 15, 3, 5, 1.1, 0);
            
      // Drawing the optical flow
    	std::vector<cv::Point2f>::const_iterator p = prev_pts.begin();
    	for(; p!=prev_pts.end(); ++p)
    	{
    		const cv::Point2f& fxy = f.at<cv::Point2f>(p->y, p->x);
        double val_flow = f.at<double>(p->y,p->x);

        // Check the value of threshold and radius
        if( val_flow > 10.0) cv::circle(org_flow, cv::Point(p->x,p->y), 1,cv::Scalar(FG), -1, 8, 0);
        // Check the value of gain ( *p+fxy*" " )
        if( val_flow > 10.0) cv::line(org_flow, *p, *p+fxy*8, cv::Scalar(FG), 1);
    	}
    	
      // Saving the image into prev_image 
      prev_image = image.clone();

      // Morphology calculation 
      cv::Mat element(3, 3, CV_8U,cv::Scalar::all(255));
      cv::Mat dilate_flow = org_flow.clone();
      cv::erode(dilate_flow, dilate_flow, cv::Mat(), cv::Point(-1,-1), 10); // Check the number of erodion

      // Labeling
      LabelingBS labeling;
      unsigned char *src = new unsigned char[image.cols*image.rows];
      short *result = new short[image.cols*image.rows];
      int l=0;
      for(int j=0 ; j<image.rows ; j++)
      {
        for(int i=0 ; i<image.cols ; i++)
        {
           src[l] = dilate_flow.at<unsigned char>(j,i);
           l++;
        }
      }
      labeling.Exec(src, result, image.cols, image.rows, true, 10);
      int nlabel = labeling.GetNumOfResultRegions();

      // Calculation of Labeling result
      RegionInfoBS *ri;
      for(int i=0 ; i<nlabel ; i++)
      {
        ri = labeling.GetResultRegionInfo(i);
        int minx,miny;
        int maxx,maxy;
        ri->GetMin(minx, miny);
        ri->GetMax(maxx, maxy);
        cv::rectangle(image, cv::Point(minx,miny), cv::Point(maxx,maxy), cv::Scalar(0), 3, 4);
      }
            
      // Display of images
      imshow("source", image);
            
      int c = waitKey(1);
     }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
    
    frame_count++;

  }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
