#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub_1;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/duckiebot4/camera_node/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/led_test/detections", 1);
    image_pub_1 = it_.advertise("/led_test/mono", 1);

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

    float start_point_pct = 0.28;
    float cropped_pct = 0.68;

    int roi_y = int(cv_ptr->image.rows * start_point_pct);
    int roi_w = cv_ptr->image.cols;
    int roi_h = int(cv_ptr->image.rows * cropped_pct);

    cv::Rect roi(0, roi_y, roi_w, roi_h);

    // param (model)
    int model_size_px = 160;

    int tg_y = int(model_size_px * start_point_pct);
    int tg_h = int(model_size_px * cropped_pct);

    cv::Rect target_roi(0, tg_y, model_size_px, tg_h);

    // member
    cv::Mat sqr_img = cv::Mat::zeros(model_size_px, model_size_px, CV_8UC3);

    cv::resize(cv_ptr->image(roi), sqr_img(target_roi), target_roi.size());


    // PEDESTRIAN

    cv::Mat lower_mask, upper_mask, red_mask, hsv_img;

    cv::cvtColor(sqr_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(0, 0, 230), cv::Scalar(13, 38, 255), lower_mask);
    cv::inRange(hsv_img, cv::Scalar(160, 0, 230), cv::Scalar(179, 38, 255), upper_mask);
    cv::bitwise_or(lower_mask, upper_mask, red_mask);
    
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(red_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    float duckie_area;
    cv::Point center;
    // std::vector<cv::Point> detections;
    std::vector< std::vector<cv::Point> > hull(contours.size());

    int duckies = 0;

    // for(int i = 0; i < contours.size(); i++){

    //   //cv::convexHull(cv::Mat(contours[i]), hull[i], false);

    //   cv::Moments M = cv::moments(contours[i]);
    //     if(M.m00 != 0){
    //       duckie_area = contourArea(contours[i]);
    //       if(1){
    //         center.x = int(M.m10 / M.m00);
    //         center.y = int(M.m01 / M.m00);
    //         //cv::circle( sqr_img, center, 1, cv::Scalar(255,0,0), 3, cv::LINE_AA);
    //         cv::drawContours(sqr_img, contours, i, cv::Scalar(255, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    //         duckies++;
    //       }          
    //     }

    // }
    for(int i = 0; i < contours.size(); i++){

      cv::drawContours(sqr_img, contours, i, cv::Scalar(255, 0, 0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

    }

    ROS_INFO("detections: %i", contours.size());
    
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sqr_img).toImageMsg();
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", red_mask).toImageMsg();

    image_pub_.publish(msg1);
    image_pub_1.publish(msg2);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "back_test");
  ImageConverter ic;
  ROS_INFO("Node started");
  ros::spin();
  return 0;
}