#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    // Initialize subscribers and publishers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::laserCallback, this, _1));
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Process laser data and determine robot's surroundings
    bool obstacle_detected = false;
    //RCLCPP_INFO(this->get_logger(), "I heard data");

    // double min_range = std::numeric_limits<double>::infinity();
    for (size_t i = 330; i < 390; ++i) {
      double range = msg->ranges[i];
      if (range > msg->range_min && range < msg->range_max &&
          range < obstacle_distance_) {
        obstacle_detected = true;
      }
    }

    // Calculate proper velocity command based on surroundings

    if (obstacle_detected) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 1.0;
    } else {
      cmd_vel.linear.x = 0.2;
      cmd_vel.angular.z = 0.0;
    }

    // Publish velocity command to cmd_vel topic
    cmd_vel_pub_->publish(cmd_vel);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Twist cmd_vel;
  double obstacle_distance_ = 0.5;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto patrol = std::make_shared<Patrol>();
  rclcpp::spin(patrol);
  rclcpp::shutdown();
  return 0;
}
