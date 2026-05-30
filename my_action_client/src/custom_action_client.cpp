#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "custom_interfaces/action/go_to_pose.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CustomActionClient : public rclcpp::Node {
public:
  using GoToPose = custom_interfaces::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

  explicit CustomActionClient(const rclcpp::NodeOptions &options)
      : Node("custom_action_client", options) {
    this->action_client_ =
        rclcpp_action::create_client<GoToPose>(this, "go_to_pose");

    // Subscribe to odometry to get the robot's position
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&CustomActionClient::odom_callback, this,
                  std::placeholders::_1));

    // Subscribe to model states to track the meteor position
    model_states_subscription_ =
        this->create_subscription<gazebo_msgs::msg::ModelStates>(
            "/gazebo/model_states", 10,
            std::bind(&CustomActionClient::model_states_callback, this,
                      std::placeholders::_1));

    // Initialize positions
    robot_x_ = std::nullopt;
    robot_y_ = std::nullopt;
    meteor_x_ = std::nullopt;
    meteor_y_ = std::nullopt;

    // Distance threshold to send new goal
    near_meteor_threshold_ = 7.5; // Units (e.g., meters)

    // Flag to indicate if new goal has been sent
    new_goal_sent_ = false;
  }

  void send_goal(double x, double y, double yaw) {
    using namespace std::placeholders;

    auto goal_msg = GoToPose::Goal();
    goal_msg.x = x;
    goal_msg.y = y;
    goal_msg.yaw = yaw;

    RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f, yaw=%.2f", x,
                y, yaw);

    if (!this->action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto send_goal_options = rclcpp_action::Client<GoToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&CustomActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&CustomActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&CustomActionClient::result_callback, this, _1);

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<GoToPose>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr
      model_states_subscription_;

  // Position variables using std::optional
  std::optional<double> robot_x_;
  std::optional<double> robot_y_;
  std::optional<double> meteor_x_;
  std::optional<double> meteor_y_;

  // Distance threshold and flags
  double near_meteor_threshold_;
  bool new_goal_sent_;

  void
  goal_response_callback(const GoalHandleGoToPose::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "Goal was rejected by the action server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    }
  }

  void
  feedback_callback(GoalHandleGoToPose::SharedPtr,
                    const std::shared_ptr<const GoToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback from action server.");
  }

  void result_callback(const GoalHandleGoToPose::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action completed with success: true");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(),
                  "Action completed with success: false (aborted)");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(),
                  "Action completed with success: false (canceled)");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update robot's position
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    // Check if we have both positions and haven't sent the new goal yet
    if (robot_x_.has_value() && robot_y_.has_value() && meteor_x_.has_value() &&
        meteor_y_.has_value() && !new_goal_sent_) {
      // Calculate distance to meteor
      double distance = std::hypot(robot_x_.value() - meteor_x_.value(),
                                   robot_y_.value() - meteor_y_.value());
      if (distance <= near_meteor_threshold_) {
        RCLCPP_INFO(this->get_logger(),
                    "Robot is near meteor. Sending new goal to avoid it.");
        // Send new goal to next coordinates of lost astronaut
        send_goal(-11.5, -3.5, -1.5);
        new_goal_sent_ = true; // Prevent sending multiple goals
      }
    }
  }

  void
  model_states_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
    // Extract the meteor's position from the ModelStates message
    auto it = std::find(msg->name.begin(), msg->name.end(), "meteor");
    if (it != msg->name.end()) {
      // Get index of the meteor in the model names list
      size_t meteor_index = std::distance(msg->name.begin(), it);

      // Get the meteor's position
      if (meteor_index < msg->pose.size()) {
        meteor_x_ = msg->pose[meteor_index].position.x;
        meteor_y_ = msg->pose[meteor_index].position.y;
      }
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_client =
      std::make_shared<CustomActionClient>(rclcpp::NodeOptions());

  // Send the initial goal
  double initial_x = -8.0;
  double initial_y = 6.0;
  double initial_yaw = 1.57;

  action_client->send_goal(initial_x, initial_y, initial_yaw);

  // Keep the node alive to receive callbacks
  rclcpp::spin(action_client);

  rclcpp::shutdown();
  return 0;
}