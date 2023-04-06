#include <custom_interfaces/srv/get_direction.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using GetDirection = custom_interfaces::srv::GetDirection;

class PatrolV2 : public rclcpp::Node {
public:
  PatrolV2() : Node("patrol_v2") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PatrolV2::laserCallback, this, _1));
    direction_client_ =
        this->create_client<custom_interfaces::srv::GetDirection>(
            "/direction_service");
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Create request for direction service
    auto request =
        std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Call direction service and process response
    direction_client_->async_send_request(
        request, std::bind(&PatrolV2::directionResponseCallback, this,
                           std::placeholders::_1));
  }

  void directionResponseCallback(
      rclcpp::Client<GetDirection>::SharedFuture response) {
    if (response.get()) {

      if (response.get()->direction == "forward") {
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.0;

      } else if (response.get()->direction == "left") {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = -1.0;

      } else {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 1.0;
      }
      cmd_vel_pub_->publish(cmd_vel);
      RCLCPP_INFO(this->get_logger(), "Robot direction: %s",
                  response.get()->direction.c_str());
    }

    else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call direction service");
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr
      direction_client_;
  geometry_msgs::msg::Twist cmd_vel;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol = std::make_shared<PatrolV2>();
  rclcpp::spin(patrol);
  rclcpp::shutdown();
  return 0;
}
