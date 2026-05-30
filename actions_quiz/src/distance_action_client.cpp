#include <functional>
#include <memory>

#include "actions_quiz_msg/action/distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class DistanceActionClient : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ClientGoalHandle<Distance>;

  explicit DistanceActionClient(const rclcpp::NodeOptions &options)
      : Node("distance_action_client", options) {
    this->action_client_ =
        rclcpp_action::create_client<Distance>(this, "distance_as");
  }

  void send_goal(double x, double y, double yaw) {
    using namespace std::placeholders;

    auto goal_msg = Distance::Goal();
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

    auto send_goal_options = rclcpp_action::Client<Distance>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DistanceActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&DistanceActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&DistanceActionClient::result_callback, this, _1);

    this->action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Distance>::SharedPtr action_client_;

  void
  goal_response_callback(const GoalHandleDistance::SharedPtr &goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(),
                   "Goal was rejected by the action server.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server.");
    }
  }

  void
  feedback_callback(GoalHandleDistance::SharedPtr,
                    const std::shared_ptr<const Distance::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: Distance to goal  = %.2f",
                feedback->distance_left);
  }

  void result_callback(const GoalHandleDistance::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Success: distance traveled = %.2f",
                  result.result->distance_traveled);
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal canceled");
      break;

    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result");
      break;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_client =
      std::make_shared<DistanceActionClient>(rclcpp::NodeOptions());

  // Send the initial goal
  double initial_x = 8.3;
  double initial_y = -2.2;
  double initial_yaw = -0.2;

  action_client->send_goal(initial_x, initial_y, initial_yaw);

  // Keep the node alive to receive callbacks
  rclcpp::spin(action_client);

  rclcpp::shutdown();
  return 0;
}