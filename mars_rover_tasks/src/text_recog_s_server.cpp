#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <regex>
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

class TextRecognitionService : public rclcpp::Node {
public:
  TextRecognitionService() : Node("text_recog_node") {

    // Create a service that will handle status queries
    std::string name_service = "/text_recognition_service";
    service_ = this->create_service<std_srvs::srv::Trigger>(
        name_service,
        std::bind(&TextRecognitionService::get_status_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    initializeTextDetection();

    // Subscribe to image topic
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/leo/camera/image_raw", 1,
        std::bind(&TextRecognitionService::image_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Text Recognition Service initialized");
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
    latest_image_msg_ = msg;
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
        detected_text_ = simulateTextRecognition(roi);

        if (!detected_text_.empty()) {
          std::string position =
              std::to_string(bounding_rect.x) + "-" +
              std::to_string(bounding_rect.y) + "-" +
              std::to_string(bounding_rect.x + bounding_rect.width) + "-" +
              std::to_string(bounding_rect.y + bounding_rect.height);

          //   RCLCPP_INFO(this->get_logger(), "OCR Result: %s, (%s)",
          //               detected_text_.c_str(), position.c_str());
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

private:
  void get_status_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Suppress unused parameter warning

    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(latest_image_msg_,
                                   sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Perform text detection
    detectText(cv_ptr->image);

    if (detected_text_ == "FOOD" || detected_text_ == "WASTE") {
      // Construct response message
      response->success = true;
      response->message = "Result: " + detected_text_;
      RCLCPP_INFO(this->get_logger(), "Result: %s", detected_text_.c_str());
    } else {
      response->success = false;
      response->message = "Result: " + detected_text_;
      RCLCPP_INFO(this->get_logger(), "Result: %s", detected_text_.c_str());
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  sensor_msgs::msg::Image::SharedPtr latest_image_msg_;
  std::string detected_text_;
  //   cv::dnn::Net net_;
  float confidence_threshold_;
  float nms_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TextRecognitionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}