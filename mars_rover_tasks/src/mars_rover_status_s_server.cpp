#include <iomanip>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>

class RobotStatusService : public rclcpp::Node {
public:
  RobotStatusService() : Node("robot_status_service") {
    // Initialize random number generator
    rng_.seed(std::random_device{}());

    // Create a service that will handle status queries
    std::string name_service = "/get_robot_status";
    service_ = this->create_service<std_srvs::srv::Trigger>(
        name_service, std::bind(&RobotStatusService::get_status_callback, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
  void get_status_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request; // Suppress unused parameter warning

    // Simulated data
    std::uniform_real_distribution<double> battery_dist(20.0, 100.0);
    std::uniform_real_distribution<double> temp_dist(25.0, 80.0);
    std::uniform_int_distribution<int> bool_dist(0, 1);

    double battery_level = battery_dist(rng_);
    double temperature = temp_dist(rng_);
    bool camera_status = bool_dist(rng_);
    std::vector<bool> motor_statuses;

    for (int i = 0; i < 4; ++i) {
      motor_statuses.push_back(bool_dist(rng_));
    }

    // Construct response message
    response->success = true;

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << "Battery Level: " << battery_level << "%, "
       << "Temperature: " << temperature << "°C, "
       << "Camera: " << (camera_status ? "Operational" : "Faulty") << ", "
       << "Motor 1: " << (motor_statuses[0] ? "Operational" : "Faulty") << ", "
       << "Motor 2: " << (motor_statuses[1] ? "Operational" : "Faulty") << ", "
       << "Motor 3: " << (motor_statuses[2] ? "Operational" : "Faulty") << ", "
       << "Motor 4: " << (motor_statuses[3] ? "Operational" : "Faulty") << ", ";

    response->message = ss.str();
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  std::mt19937 rng_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStatusService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}