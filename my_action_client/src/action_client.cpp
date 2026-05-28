#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "leo_description/action/rotate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MyActionClient : public rclcpp::Node
{
public:
  using Rotate = leo_description::action::Rotate;
  using GoalHandleRotate = rclcpp_action::ClientGoalHandle<Rotate>;

  explicit MyActionClient(const rclcpp::NodeOptions & options)
  : Node("my_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Rotate>(
      this,
      "rotate");
  }

  void send_goal(double seconds)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Rotate::Goal();
    goal_msg.rotation_time = seconds;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Rotate>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&MyActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&MyActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&MyActionClient::result_callback, this, _1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Rotate>::SharedPtr client_ptr_;

  void goal_response_callback(const GoalHandleRotate::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal rejected :(");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
    }
  }

  void feedback_callback(
    GoalHandleRotate::SharedPtr,
    const std::shared_ptr<const Rotate::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Received feedback: " << feedback->elapsed_time;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleRotate::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result: " << (result.result->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_client = std::make_shared<MyActionClient>(rclcpp::NodeOptions());

  action_client->send_goal(5.0);

  rclcpp::spin(action_client);

  return 0;
}
