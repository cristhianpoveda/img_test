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

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/duckiebot4/camera_node/image", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

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

    cv::Mat yellow_mask, hsv_img;

    cv::cvtColor(sqr_img, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(28, 38, 37), cv::Scalar(40, 153, 57), yellow_mask);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(yellow_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    size_t contour_len = sizeof(contours) / sizeof(contours[0]);

    float duckie_area;
    cv::Point center;
    std::vector<cv::Point> detections;

    for(size_t i = 0; i < contour_len; i++){
      duckie_area = cv::contourArea(contours[i]);
      if(duckie_area > 9){
        
        cv::Moments M = cv::moments(contours[i]);
        if(M.m00 != 0){
          center.x = int(M.m10 / M.m00);
          center.y = int(M.m01 / M.m00);
        }else{
          center.x = 0;
          center.y = 0;
        }
        ROS_INFO("x: %d", center.x);
        ROS_INFO("y: %d", center.y);
        detections.push_back(center);
        cv::circle( sqr_img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
      }

      size_t detection_len = sizeof(detections) / sizeof(detections[0]);
      
    }

    // cv::drawContours( sqr_img, contours, -1, cv::Scalar(0,255,0), cv::FILLED, cv::LINE_8, hierarchy, 1);

    // ROS_INFO("rows %i", yellow_mask.rows);
    // ROS_INFO("cols %i", yellow_mask.cols);
    // ROS_INFO("ch %i", yellow_mask.channels());
    
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sqr_img).toImageMsg();

    image_pub_.publish(msg1);
    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testn");
  ImageConverter ic;
  ROS_INFO("Node started");
  ros::spin();
  return 0;
}