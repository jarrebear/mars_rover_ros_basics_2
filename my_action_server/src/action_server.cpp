#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "action_msgs/msg/goal_status.hpp"
#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MyActionServer : public rclcpp::Node {
public:
  using GoToPose = custom_interfaces::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_action_server", options) {
    using namespace std::placeholders;

    // Action server to accept goals
    this->action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    // Publisher to set initial pose
    initial_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

    // Action client for navigation to pose
    nav_to_pose_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    // Wait for the localization node to be ready
    wait_for_localization();

    // Set the initial pose to (0, 0, 0)
    set_initial_pose(0.0, 0.0, 0.0);
  }

private:
  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_publisher_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  void wait_for_localization() {
    RCLCPP_INFO(this->get_logger(), "Waiting for localization to be active...");

    // Wait for subscribers to /initialpose
    while (this->count_subscribers("/initialpose") == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for subscribers to /initialpose...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Wait for /amcl_pose topic
    while (!topic_exists("/amcl_pose")) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /amcl_pose topic...");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(this->get_logger(), "Localization is active.");
  }

  bool topic_exists(const std::string &topic_name) {
    auto topics = this->get_topic_names_and_types();
    for (const auto &topic : topics) {
      if (topic.first == topic_name) {
        return true;
      }
    }
    return false;
  }

  void set_initial_pose(double x, double y, double yaw) {
    auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->get_clock()->now();

    initial_pose.pose.pose.position.x = x;
    initial_pose.pose.pose.position.y = y;

    // Set the orientation as quaternion (for yaw rotation)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    initial_pose.pose.pose.orientation = tf2::toMsg(q);

    // Publish the initial pose multiple times
    for (int i = 0; i < 10; ++i) {
      initial_pose_publisher_->publish(initial_pose);
      RCLCPP_INFO(this->get_logger(), "Publishing initial pose (%d/10)", i + 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(),
                "Initial pose set to x: %.2f, y: %.2f, yaw: %.2f", x, y, yaw);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %.2f, y: %.2f, yaw: %.2f",
                goal->x, goal->y, goal->yaw);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    feedback->feedback = "Navigating to goal...";
    goal_handle->publish_feedback(feedback);

    // Send navigation goal
    bool navigation_result = send_navigation_goal(goal->x, goal->y, goal->yaw);

    if (navigation_result) {
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
      result->success = true;
      goal_handle->succeed(result);
    } else {
      RCLCPP_INFO(this->get_logger(), "Failed to reach goal :(");
      result->success = false;
      goal_handle->abort(result);
    }
  }

  bool send_navigation_goal(double x, double y, double yaw) {
    auto goal_msg = NavigateToPose::Goal();

    // Set goal position
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;

    // Set goal orientation (yaw as quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    // Set the frame and timestamp
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();

    // Wait for the NavigateToPose action server
    RCLCPP_INFO(this->get_logger(),
                "Waiting for the NavigateToPose action server...");
    if (!nav_to_pose_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "NavigateToPose action server not available");
      return false;
    }

    // Send the goal
    RCLCPP_INFO(this->get_logger(),
                "Sending navigation goal to: x=%.2f, y=%.2f, yaw=%.2f", x, y,
                yaw);
    auto send_goal_future = nav_to_pose_client_->async_send_goal(goal_msg);

    // Wait for goal to be accepted
    if (send_goal_future.wait_for(std::chrono::seconds(10)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send goal");
      return false;
    }

    auto nav_goal_handle = send_goal_future.get();
    if (!nav_goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Navigation goal rejected");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Navigation goal accepted");

    // Wait for result
    auto get_result_future =
        nav_to_pose_client_->async_get_result(nav_goal_handle);

    get_result_future.wait();

    auto nav_result = get_result_future.get();
    if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Navigation failed with status: %d",
                  static_cast<int>(nav_result.code));
      return false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}