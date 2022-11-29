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

    cv::cvtColor(sqr_img, sqr_img, cv::COLOR_BGR2HSV);

    cv::Mat yellow_mask;

    cv::inRange(sqr_img, cv::Scalar(25, 140, 100), cv::Scalar(45, 255, 255), yellow_mask);

    ROS_INFO("rows %i", yellow_mask.rows);
    ROS_INFO("cols %i", yellow_mask.cols);
    ROS_INFO("ch %i", yellow_mask.channels());
    
    sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "mono8", yellow_mask).toImageMsg();

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