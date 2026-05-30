#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

class MyActionServer : public rclcpp::Node {
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;
  //   using GoToPose = leo_description::action::GoToPose;
  //   using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MyActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("quiz_action_server", options) {
    using namespace std::placeholders;

    // Action server to accept goals provide total distance travelled and give
    // feedback of distance left
    this->action_server_ = rclcpp_action::create_server<Distance>(
        this, "/distance_as",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    // Publisher to set initial pose
    initial_pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

    // Publisher for remaining distance
    remaining_distance_publisher_ =
        this->create_publisher<std_msgs::msg::Float64>("/distance_left", 10);

    // Publisher for total distance traveled
    total_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
        "/distance_traveled", 10);

    // Subscriber for odometry

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MyActionServer::odom_callback, this, std::placeholders::_1));

    // Action client for navigation to pose
    nav_to_pose_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    // Wait for the localization node to be ready
    wait_for_localization();

    // Set the initial pose to (0, 0, 0)
    set_initial_pose(0.0, 0.0, 0.0);
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      remaining_distance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
      total_distance_publisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;

  // Initialize our variables
  bool first_odom_ = true;
  double distance_traveled_{0};
  double previous_x_;
  double previous_y_;
  double current_position_x_;
  double current_position_y_;
  bool nav_success_ = false;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract the x, y coordinates from the odometry message
    current_position_x_ = msg->pose.pose.position.x;
    current_position_y_ = msg->pose.pose.position.y;
    auto pub_distance = std_msgs::msg::Float64();
    if (first_odom_) {
      first_odom_ = false;
      previous_x_ = current_position_x_;
      previous_y_ = current_position_y_;
      pub_distance.data = 0.0;
    } else {
      distance_traveled_ += calculate_distance(
          current_position_x_, current_position_y_, previous_x_, previous_y_);
      previous_x_ = current_position_x_;
      previous_y_ = current_position_y_;
      pub_distance.data = distance_traveled_;
    }
    total_distance_publisher_->publish(pub_distance);
  }

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
              std::shared_ptr<const Distance::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %.2f, y: %.2f, yaw: %.2f",
                goal->x, goal->y, goal->yaw);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle) {
    nav_success_ = false;
    first_odom_ = true;
    distance_traveled_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto result = std::make_shared<Distance::Result>();

    // feedback->feedback = "Navigating to goal...";
    // goal_handle->publish_feedback(feedback);

    // Send navigation goal
    send_navigation_goal(goal->x, goal->y, goal->yaw);

    rclcpp::Rate rate(10);
    auto distance_left_msg = std_msgs::msg::Float64();
    double distance_to_goal{0.0};
    while (rclcpp::ok()) {

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        return;
      }

      distance_to_goal = calculate_distance(
          current_position_x_, current_position_y_, goal->x, goal->y);
      RCLCPP_INFO(this->get_logger(), "Feedback: Distance left = %.2f",
                  distance_to_goal);
      feedback->distance_left = distance_to_goal;
      distance_left_msg.data = distance_to_goal;
      remaining_distance_publisher_->publish(distance_left_msg);
      goal_handle->publish_feedback(feedback);

      if (nav_success_)
        break;

      rate.sleep();
    }

    result->distance_traveled = distance_traveled_;
    result->success = nav_success_;

    if (nav_success_) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
    RCLCPP_INFO(this->get_logger(), "Action completed with success: %s\n",
                nav_success_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Total distance travelled = %.2f",
                distance_traveled_);
  }

  void send_navigation_goal(double x, double y, double yaw) {
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
      return;
    }

    // Send the goal
    RCLCPP_INFO(this->get_logger(),
                "Sending navigation goal to: x=%.2f, y=%.2f, yaw=%.2f", x, y,
                yaw);

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    // send_goal_options.goal_response_callback =
    //     std::bind(&MyActionClient::goal_response_callback, this, _1);
    // send_goal_options.feedback_callback =
    //     std::bind(&MyActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(
        &MyActionServer::result_callback, this, std::placeholders::_1);

    this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      nav_success_ = true;
      RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
    } else {
      nav_success_ = false;
      RCLCPP_ERROR(this->get_logger(), "Navigation failed");
    }
  }

  double calculate_distance(double current_position_x,
                            double current_position_y, double previous_x,
                            double previous_y) {
    double dx = current_position_x - previous_x;
    double dy = current_position_y - previous_y;

    return std::sqrt(dx * dx + dy * dy);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}