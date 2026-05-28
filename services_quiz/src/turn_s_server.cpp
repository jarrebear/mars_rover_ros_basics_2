#include "rclcpp/publisher.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <services_quiz_srv/srv/turn.hpp>

// Use the data received in the request (request) part of the message to make
// the robot spin:

//     The robot will spin to one side or another depending on the direction
//     value. Use the angular_velocity value to determine the velocity at which
//     the robot will spin. Use the time value to determine the duration of the
//     spin. Finally, it must return True if everything went okay in the success
//     variable, which in this case is simply that the service server callback
//     completes.

class TurnService : public rclcpp::Node {
public:
  TurnService() : Node("turn_server_node") {

    // Create a service that will handle turn commands
    std::string name_service = "/turn";
    turn_service_ = this->create_service<services_quiz_srv::srv::Turn>(
        name_service, std::bind(&TurnService::turn_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    // Publisher for movement commands
    publisher_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Turn Service initialized");
  }

private:
  void turn_callback(
      const std::shared_ptr<services_quiz_srv::srv::Turn::Request> request,
      std::shared_ptr<services_quiz_srv::srv::Turn::Response> response) {

    // Read in values from our service request
    std::string direction = request->direction;
    double angular_velocity = request->angular_velocity;
    double turn_time = request->time;

    // Generate our message to publish
    auto action = geometry_msgs::msg::Twist();

    // Based on turn direction give action proper angular velocity
    if (direction == "left") {
      action.angular.z = angular_velocity;
    } else if (direction == "right") {
      action.angular.z = -angular_velocity;
    } else {
      RCLCPP_INFO(this->get_logger(), "Invalid turn direction");
      response->success = false;
      return;
    }

    rclcpp::Duration duration = rclcpp::Duration::from_seconds(turn_time);
    auto start_time = this->now();

    rclcpp::Rate rate(10); // 10 Hz

    // Publish for set time at 10 Hz
    while ((this->now() - start_time) < duration) {
      publisher_vel_->publish(action);
      rate.sleep();
    }

    // Halt after set time and give the success response
    publisher_vel_->publish(geometry_msgs::msg::Twist());
    response->success = true;
  }

private:
  rclcpp::Service<services_quiz_srv::srv::Turn>::SharedPtr turn_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurnService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}