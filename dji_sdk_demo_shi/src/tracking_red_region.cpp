#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int h_low;
  int h_high;
  int s_low;
  int s_high;
  int v_low;
  int v_high;
  int switch_value;
  int settings1;
  int settings2;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/left/image_rect_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);

    //HSV 101,30,165 - 255,255,255
    h_low = 101;
    s_low = 30;
    v_low = 165;
    h_high = 255;
    s_high = 255;
    v_high = 255;
    // h_low = 0;
    // s_low = 0;
    // v_low = 190;
    // h_high = 255;
    // s_high = 255;
    // v_high = 255;
    switch_value = 0;
    settings1 = 100;
    settings2 = 20;
      
    cv::createTrackbar("h_low", "Image window", &h_low, 255);
    cv::createTrackbar("s_low", "Image window", &s_low, 255);
    cv::createTrackbar("v_low", "Image window", &v_low, 255);
    cv::createTrackbar("h_high", "Image window", &h_high, 255);
    cv::createTrackbar("s_high", "Image window", &s_high, 255);
    cv::createTrackbar("v_high", "Image window", &v_high, 255);
    cv::createTrackbar("switch", "Image window", &switch_value, 2);
    cv::createTrackbar("settings1", "Image window", &settings1, 1000);
    cv::createTrackbar("settings2", "Image window", &settings2, 1000);
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

    cv::Mat original_image, hsv_image, threshold_image;;

    //Image processing
    cv_ptr->image.copyTo(original_image);

    //Read local image
    //cv::Mat src_image = cv::imread("/home/shi/image3.png");
    //src_image.copyTo(original_image);

    cv::cvtColor(original_image, hsv_image, CV_RGB2HSV);
    //original_image.copyTo(hsv_image);
    
    cv::inRange(hsv_image, cv::Scalar(h_low,s_low,v_low), cv::Scalar(h_high,s_high,v_high), threshold_image);

    if(switch_value == 2)
      cv::fastNlMeansDenoising(threshold_image, threshold_image);
    else if(switch_value == 1)
      cv::GaussianBlur(threshold_image, threshold_image, cv::Size(15,15),2,2);
    
    cv::dilate(threshold_image, threshold_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
    cv::erode(threshold_image, threshold_image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(50, 50)) );

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    cv::HoughCircles(threshold_image, circles, CV_HOUGH_GRADIENT, 1, 20, settings1, settings2, 0, 0 );

    cv::Mat threshold_rgb_image;
    cv::cvtColor(threshold_image, threshold_rgb_image, CV_GRAY2RGB);
    /// Draw the circles detected
    std::cout << circles.size() << '\n';
    for( size_t i = 0; i < circles.size(); i++ )
      {
	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	int radius = cvRound(circles[i][2]);
	// circle center
	circle(threshold_rgb_image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	// circle outline
	circle(threshold_rgb_image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
      }
    
    
    //cv::fastNlMeansDenoising(threshold_image, threshold_image);
    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(OPENCV_WINDOW, threshold_rgb_image);
    // cv::imshow(OPENCV_WINDOW, src_image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

