#include <rclcpp/rclcpp.hpp>
#include <services_quiz_srv/srv/turn.hpp>

#include <chrono>

using namespace std::chrono_literals;

class TurnClient : public rclcpp::Node {
public:
  TurnClient() : Node("turn_client_node") {
    // Create the Service Client object
    std::string name_service = "/turn";
    client_ = this->create_client<services_quiz_srv::srv::Turn>(name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }
  }

  void send_request() {

    // Create a TurnService request
    auto request = std::make_shared<services_quiz_srv::srv::Turn::Request>();
    request->direction = "right";
    request->angular_velocity = 0.2;
    request->time = 10.0;

    // Send the request asynchronously
    auto result_future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      // Log the service response
      RCLCPP_INFO(this->get_logger(), "Result: %s",
                  response->success ? "true" : "false");

    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

private:
  rclcpp::Client<services_quiz_srv::srv::Turn>::SharedPtr client_;
};

int main(int argc, char **argv) {
  // Initialize the ROS communication
  rclcpp::init(argc, argv);

  // Declare the node constructor
  auto client = std::make_shared<TurnClient>();

  // Run the send_request() method
  client->send_request();

  // Shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}