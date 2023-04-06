#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_server_node") {
    srv_ = create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::moving_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void moving_callback(const std::shared_ptr<GetDirection::Request> request,
                       const std::shared_ptr<GetDirection::Response> response) {

    auto msg = request->laser_data;
    double min_forward = std::numeric_limits<double>::infinity();
    auto min_right = min_forward;
    auto min_left = min_forward;
    double range = 0;

    for (size_t i = 180; i < 300; ++i) {
      range = msg.ranges[i];
      if (range < min_left && range > msg.range_min && range < msg.range_max) {
        min_left = range;
      }
    }

    for (size_t i = 300; i < 420; ++i) {
      range = msg.ranges[i];
      if (range < min_forward && range > msg.range_min && range < msg.range_max) {
        min_forward = range;
      }
    }

    for (size_t i = 420; i < 540; ++i) {
      range = msg.ranges[i];
      if (range < min_right && range > msg.range_min && range < msg.range_max) {
        min_right = range;
      }
    }
    if (min_forward > min_left && min_forward > min_right) {
      response->direction = "forward";
    } else if (min_left > min_right) {
      response->direction = "left";
    } else {
      response->direction = "right";
    };
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}