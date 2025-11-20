#ifndef VIDEO_PROCESSOR__VIDEO_PROCESSOR_NODE_HPP_
#define VIDEO_PROCESSOR__VIDEO_PROCESSOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <auto_aim_interfaces/msg/armors.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>

namespace video_processor
{

class VideoProcessorNode : public rclcpp::Node
{
public:
  explicit VideoProcessorNode(const rclcpp::NodeOptions & options);
  ~VideoProcessorNode();

private:
  void processVideo();
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg);
  void drawArmors(cv::Mat & frame, const auto_aim_interfaces::msg::Armors::SharedPtr & armors);
  std::string getColorName(uint8_t color);
  
  // OpenCV video capture and writer
  std::unique_ptr<cv::VideoCapture> video_capture_;
  std::unique_ptr<cv::VideoWriter> video_writer_;
  
  // ROS2 publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Armors>::SharedPtr armors_sub_;
  
  // Timer for video processing
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameters
  std::string input_video_path_;
  std::string output_video_path_;
  double playback_speed_;
  int fps_;
  bool show_preview_;
  
  // Frame info
  std::string frame_id_;
  int frame_width_;
  int frame_height_;
  
  // Latest detection results
  auto_aim_interfaces::msg::Armors::SharedPtr latest_armors_;
  std::mutex armors_mutex_;
  
  // Statistics
  int total_frames_;
  int processed_frames_;
  int detected_frames_;
};

}  // namespace video_processor

#endif  // VIDEO_PROCESSOR__VIDEO_PROCESSOR_NODE_HPP_
