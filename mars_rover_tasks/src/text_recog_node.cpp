#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <sensor_msgs/msg/image.hpp>

class TextRecognitionNode : public rclcpp::Node {
public:
  TextRecognitionNode() : Node("text_recognition_node") {
    // Initialize text detection (simplified version)
    initializeTextDetection();

    // Subscribe to image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/leo/camera/image_raw", 1,
        std::bind(&TextRecognitionNode::image_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Text Recognition Node initialized");
  }

private:
  void initializeTextDetection() {
    // For a complete implementation, you would load the EAST model here:
    // net_ =
    // cv::dnn::readNet("/home/user/ros2_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb");

    // For now, we'll use a simplified approach
    confidence_threshold_ = 0.5;
    nms_threshold_ = 0.4;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Perform text detection
    detectText(cv_ptr->image);
  }

  void detectText(const cv::Mat &image) {
    // Simplified text detection using basic OpenCV methods
    // In a full implementation, you would use the EAST model or integrate the
    // Python module

    cv::Mat gray, thresh;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Find contours that might contain text
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    for (const auto &contour : contours) {
      cv::Rect bounding_rect = cv::boundingRect(contour);

      // Filter by size (assuming text boxes have reasonable dimensions)
      if (bounding_rect.width > 50 && bounding_rect.height > 20 &&
          bounding_rect.width < 200 && bounding_rect.height < 100) {

        // Extract ROI
        cv::Mat roi = image(bounding_rect);

        // Simulate text recognition (replace with actual OCR)
        std::string detected_text = simulateTextRecognition(roi);

        if (!detected_text.empty()) {
          std::string position =
              std::to_string(bounding_rect.x) + "-" +
              std::to_string(bounding_rect.y) + "-" +
              std::to_string(bounding_rect.x + bounding_rect.width) + "-" +
              std::to_string(bounding_rect.y + bounding_rect.height);

          RCLCPP_INFO(this->get_logger(), "OCR Result: %s, (%s)",
                      detected_text.c_str(), position.c_str());
        }
      }
    }
  }

  std::string simulateTextRecognition(const cv::Mat &roi) {
    // This is a simplified simulation - replace with actual OCR
    // For a complete solution, integrate Tesseract OCR or the Python model

    // Analyze color patterns to guess the text
    cv::Scalar mean_color = cv::mean(roi);

    // Simple heuristic based on the expected label colors
    // This is just for demonstration - use proper OCR in production
    if (mean_color[1] > mean_color[0] && mean_color[1] > mean_color[2]) {
      return "FOOD"; // Greenish tint might indicate FOOD label
    } else if (mean_color[0] > mean_color[1] && mean_color[2] < mean_color[0]) {
      return "WASTE"; // Reddish tint might indicate WASTE label
    }

    return ""; // No text detected
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  cv::dnn::Net net_;
  float confidence_threshold_;
  float nms_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TextRecognitionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}