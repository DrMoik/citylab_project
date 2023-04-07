#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/action/go_to_point.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GoToPoint : public rclcpp::Node {
public:
  using G2P = custom_interfaces::action::GoToPoint;
  using GoalHandleG2P = rclcpp_action::ServerGoalHandle<G2P>;

  explicit GoToPoint(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<G2P>(
        this, "go_to_point", std::bind(&GoToPoint::handle_goal, this, _1, _2),
        std::bind(&GoToPoint::handle_cancel, this, _1),
        std::bind(&GoToPoint::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPoint::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<G2P>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  nav_msgs::msg::Odometry current_odom_;

  //***************************************************
  //************************Methods********************
  //***************************************************

  //*******************Odometry Subscriber Callbacks*********

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
  }
  //*******************Actions Server Callbacks*********
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const G2P::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position x=%f, y=%f th=%f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleG2P> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleG2P> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPoint::execute, this, _1), goal_handle}.detach();
  }

  //*******************************Action Execution________________
  void execute(const std::shared_ptr<GoalHandleG2P> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<G2P::Feedback>();
    // auto &message = feedback->current_pos;
    //  message = "Starting movement...";
    auto result = std::make_shared<G2P::Result>();
    auto cmd_vel = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(10);
    //****

    while (rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        publisher_->publish(cmd_vel);
        return;
      }

      // Calculate the error between the current position and the goal position
      double error_x = goal->goal_pos.x - current_odom_.pose.pose.position.x;
      double error_y = goal->goal_pos.y - current_odom_.pose.pose.position.y;

      // Calculate the distance to the goal position
      double distance = std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2));

      // Calculate the angle between the current position and the goal position

      double siny_cosp = 2 * (current_odom_.pose.pose.orientation.w *
                                  current_odom_.pose.pose.orientation.z +
                              current_odom_.pose.pose.orientation.x *
                                  current_odom_.pose.pose.orientation.y);
      double cosy_cosp = 1 - 2 * (current_odom_.pose.pose.orientation.y *
                                      current_odom_.pose.pose.orientation.y +
                                  current_odom_.pose.pose.orientation.z *
                                      current_odom_.pose.pose.orientation.z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);

      feedback->current_pos.x = current_odom_.pose.pose.position.x;
      feedback->current_pos.y = current_odom_.pose.pose.position.y;
      feedback->current_pos.z = yaw;

      double angle = std::atan2(error_y, error_x) - yaw;

      // Check if we've reached the goal position
      if (distance < 0.05 && std::abs(goal->goal_pos.z - yaw) < 0.05) {
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        publisher_->publish(cmd_vel);
        return;
      } else if (distance < 0.05 && std::abs(goal->goal_pos.z - yaw) > 0.05) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
      }

      if (std::abs(angle) > 0.05) {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = std::min(std::max(0.5 * angle, -1.0), 1.0);
      } else {
        cmd_vel.linear.x = std::min(std::max(0.3 * distance, -1.0), 1.0);
        cmd_vel.angular.z = 0.0;
      }

      // Publish the velocity command and feedback
      publisher_->publish(cmd_vel);
      goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
  }
}; // class GoToPoint

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoint>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}