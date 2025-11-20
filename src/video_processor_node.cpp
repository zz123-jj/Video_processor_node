#include "video_processor/video_processor_node.hpp"
#include <chrono>
#include <thread>

namespace video_processor
{

VideoProcessorNode::VideoProcessorNode(const rclcpp::NodeOptions & options)
: Node("video_processor", options),
  total_frames_(0),
  processed_frames_(0),
  detected_frames_(0)
{
  RCLCPP_INFO(this->get_logger(), "Initializing VideoProcessorNode...");
  
  // Declare and get parameters
  input_video_path_ = this->declare_parameter("input_video_path", "");
  output_video_path_ = this->declare_parameter("output_video_path", "output_annotated.mp4");
  playback_speed_ = this->declare_parameter("playback_speed", 1.0);
  show_preview_ = this->declare_parameter("show_preview", false);
  frame_id_ = this->declare_parameter("frame_id", "camera_optical_frame");
  
  if (input_video_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "input_video_path parameter is empty!");
    throw std::runtime_error("input_video_path must be specified");
  }
  
  // Open input video
  video_capture_ = std::make_unique<cv::VideoCapture>(input_video_path_);
  if (!video_capture_->isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", input_video_path_.c_str());
    throw std::runtime_error("Failed to open video file");
  }
  
  // Get video properties
  fps_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FPS));
  frame_width_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FRAME_WIDTH));
  frame_height_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FRAME_HEIGHT));
  total_frames_ = static_cast<int>(video_capture_->get(cv::CAP_PROP_FRAME_COUNT));
  
  RCLCPP_INFO(this->get_logger(), "Video info: %dx%d @ %d fps, total frames: %d",
              frame_width_, frame_height_, fps_, total_frames_);
  
  // Initialize video writer
  int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
  video_writer_ = std::make_unique<cv::VideoWriter>(
    output_video_path_, fourcc, fps_, cv::Size(frame_width_, frame_height_));
    
  if (!video_writer_->isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open video writer: %s", output_video_path_.c_str());
    throw std::runtime_error("Failed to create output video file");
  }
  
  // Create publishers with best effort QoS to match detector
  rclcpp::QoS qos(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", qos);
  camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", qos);
  
  // Create subscriber for detection results with best effort QoS
  armors_sub_ = this->create_subscription<auto_aim_interfaces::msg::Armors>(
    "/detector/armors", qos,
    std::bind(&VideoProcessorNode::armorsCallback, this, std::placeholders::_1));
  
  // Create timer for video processing
  auto timer_interval = std::chrono::milliseconds(static_cast<int>(1000.0 / (fps_ * playback_speed_)));
  timer_ = this->create_wall_timer(
    timer_interval,
    std::bind(&VideoProcessorNode::processVideo, this));
  
  RCLCPP_INFO(this->get_logger(), "VideoProcessorNode initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Processing video at %.2fx speed", playback_speed_);
  RCLCPP_INFO(this->get_logger(), "Output will be saved to: %s", output_video_path_.c_str());
}

VideoProcessorNode::~VideoProcessorNode()
{
  if (video_capture_) {
    video_capture_->release();
  }
  if (video_writer_) {
    video_writer_->release();
  }
  
  RCLCPP_INFO(this->get_logger(), "Video processing completed!");
  RCLCPP_INFO(this->get_logger(), "Total frames: %d, Processed: %d, Detected: %d (%.2f%%)",
              total_frames_, processed_frames_, detected_frames_,
              processed_frames_ > 0 ? (detected_frames_ * 100.0 / processed_frames_) : 0.0);
}

void VideoProcessorNode::processVideo()
{
  cv::Mat frame;
  
  if (!video_capture_->read(frame)) {
    RCLCPP_INFO(this->get_logger(), "End of video reached");
    timer_->cancel();
    rclcpp::shutdown();
    return;
  }
  
  processed_frames_++;
  
  // Convert frame to ROS message and publish
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = this->now();
  msg->header.frame_id = frame_id_;
  image_pub_->publish(*msg);
  
  // Publish camera info with proper intrinsic parameters
  auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info_msg->header = msg->header;
  camera_info_msg->width = frame_width_;
  camera_info_msg->height = frame_height_;
  
  // Set distortion model
  camera_info_msg->distortion_model = "plumb_bob";
  
  // Set camera matrix (K) - estimated values for typical camera
  // [fx  0  cx]
  // [ 0 fy  cy]
  // [ 0  0   1]
  double fx = frame_width_ * 0.8;  // Typical focal length
  double fy = frame_width_ * 0.8;
  double cx = frame_width_ / 2.0;
  double cy = frame_height_ / 2.0;
  
  camera_info_msg->k = {
    fx,  0.0, cx,
    0.0, fy,  cy,
    0.0, 0.0, 1.0
  };
  
  // Set rectification matrix (identity for monocular camera)
  camera_info_msg->r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
  
  // Set projection matrix
  camera_info_msg->p = {
    fx,  0.0, cx,  0.0,
    0.0, fy,  cy,  0.0,
    0.0, 0.0, 1.0, 0.0
  };
  
  // Set distortion coefficients (assume no distortion)
  camera_info_msg->d = {0.0, 0.0, 0.0, 0.0, 0.0};
  
  camera_info_pub_->publish(*camera_info_msg);
  
  // Give detector some time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // Draw detection results on frame
  {
    std::lock_guard<std::mutex> lock(armors_mutex_);
    if (latest_armors_) {
      drawArmors(frame, latest_armors_);
      if (!latest_armors_->armors.empty()) {
        detected_frames_++;
      }
    }
  }
  
  // Add frame counter
  cv::putText(frame, 
              cv::format("Frame: %d/%d (%.1f%%)", processed_frames_, total_frames_,
                        processed_frames_ * 100.0 / total_frames_),
              cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 255, 255), 2);
  
  // Write annotated frame to output video
  video_writer_->write(frame);
  
  // Show preview if enabled
  if (show_preview_) {
    cv::imshow("Video Processor Preview", frame);
    cv::waitKey(1);
  }
  
  // Log progress every 100 frames
  if (processed_frames_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "Progress: %d/%d frames (%.1f%%), Detected: %d",
                processed_frames_, total_frames_,
                processed_frames_ * 100.0 / total_frames_, detected_frames_);
  }
}

void VideoProcessorNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(armors_mutex_);
  latest_armors_ = msg;
}

void VideoProcessorNode::drawArmors(
  cv::Mat & frame,
  const auto_aim_interfaces::msg::Armors::SharedPtr & armors)
{
  if (!armors || armors->armors.empty()) {
    return;
  }
  
  for (const auto & armor : armors->armors) {
    // Determine color based on armor color
    cv::Scalar box_color;
    if (armor.color == 0) {  // BLUE
      box_color = cv::Scalar(255, 0, 0);  // Blue in BGR
    } else if (armor.color == 1) {  // RED
      box_color = cv::Scalar(0, 0, 255);  // Red in BGR
    } else {
      box_color = cv::Scalar(0, 255, 0);  // Green for others
    }
    
    // Project 3D points to 2D (simplified - just draw rectangle around armor)
    // In a real implementation, you would use the pose to project the 3D corners
    
    // For now, let's draw a circle at the center
    // Convert 3D position to 2D (this is a simplified projection)
    // You should use proper camera intrinsics for accurate projection
    double fx = 1000.0;  // Focal length (should come from camera_info)
    double fy = 1000.0;
    double cx = frame_width_ / 2.0;
    double cy = frame_height_ / 2.0;
    
    // Project center point
    if (armor.pose.position.z > 0.1) {  // Avoid division by zero
      int x = static_cast<int>(armor.pose.position.x * fx / armor.pose.position.z + cx);
      int y = static_cast<int>(armor.pose.position.y * fy / armor.pose.position.z + cy);
      
      // Check if point is within frame
      if (x >= 0 && x < frame_width_ && y >= 0 && y < frame_height_) {
        // Draw circle at center
        cv::circle(frame, cv::Point(x, y), 10, box_color, 2);
        
        // Draw cross
        cv::line(frame, cv::Point(x - 15, y), cv::Point(x + 15, y), box_color, 2);
        cv::line(frame, cv::Point(x, y - 15), cv::Point(x, y + 15), box_color, 2);
        
        // Draw label
        std::string label = cv::format("%s %s d=%.2fm",
                                       getColorName(armor.color).c_str(),
                                       armor.number.c_str(),
                                       armor.distance_to_image_center);
        
        cv::putText(frame, label, cv::Point(x + 15, y - 15),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2);
      }
    }
  }
  
  // Draw detection count
  cv::putText(frame, 
              cv::format("Detected: %zu armors", armors->armors.size()),
              cv::Point(10, frame_height_ - 20),
              cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(0, 255, 0), 2);
}

std::string VideoProcessorNode::getColorName(uint8_t color)
{
  switch (color) {
    case 0:
      return "BLUE";
    case 1:
      return "RED";
    case 2:
      return "NONE";
    case 3:
      return "PURPLE";
    default:
      return "UNKNOWN";
  }
}

}  // namespace video_processor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(video_processor::VideoProcessorNode)
