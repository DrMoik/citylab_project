#include <chrono>
#include <custom_interfaces/srv/get_direction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&TestService::laserCallback, this, std::placeholders::_1));
    direction_client_ =
        this->create_client<custom_interfaces::srv::GetDirection>(
            "/direction_service");
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Create request for direction service
    auto request =
        std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Call direction service and process response
    direction_client_->async_send_request(
      request,
      std::bind(&TestService::directionResponseCallback, this, std::placeholders::_1)
    );
  }

  void directionResponseCallback(rclcpp::Client<GetDirection>::SharedFuture response) {
    if (response.get()) {
      RCLCPP_INFO(this->get_logger(), "Robot direction: %s",
                  response.get()->direction.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call direction service");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr
      direction_client_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto test_service = std::make_shared<TestService>();
  rclcpp::spin(test_service);
  rclcpp::shutdown();
  return 0;
}
