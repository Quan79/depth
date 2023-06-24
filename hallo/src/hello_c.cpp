#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
  // 将ROS图像消息转换为OpenCV图像（彩色图像和深度图像）
  cv_bridge::CvImagePtr cv_color_ptr, cv_depth_ptr;
  try
  {
    cv_color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
    cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // 保存彩色图像
  cv::Mat color_image = cv_color_ptr->image;
  std::string color_file_name = "/photo/color_image.png"; // 替换为实际的文件路径和名称
  cv::imwrite(color_file_name, color_image);
  ROS_INFO("Saved color image.");

  // 保存深度图像
  cv::Mat depth_image = cv_depth_ptr->image;
  std::string depth_file_name = "/photo/depth_image.png"; // 替换为实际的文件路径和名称
  cv::imwrite(depth_file_name, depth_image);
  ROS_INFO("Saved depth image.");

  // 可以在这里进行其他对彩色图像和深度图像的处理或分析操作

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver");
  ros::NodeHandle nh;

  // 创建彩色图像和深度图像的订阅器
  message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, "/camera_front/color/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera_front/depth/image_rect_raw", 1);

  // 定义同步策略（近似时间同步）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), color_sub, depth_sub);
  sync.registerCallback(boost::bind(&imageCallback, _1, _2));

  ros::spin();

  return 0;
}

