/*
 * Mobile Robot Closed Loop Odometry Controller, ROS2
 * Author: Roberto Zegers R.
 * License: BSD-3-Clause
 * Date: Sept, 2022
 * Initial version inspired by this ROS1 tutorial: Using the base controller
 * with odometry and transform information
 * https://wiki.ros.org/pr2_controllers/Tutorials/
 */

#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

class ClosedLoopOdom : public rclcpp::Node {
public:
  ClosedLoopOdom() : Node("heading_angle_node") {
    pub_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Call on_timer function at 2Hz
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ClosedLoopOdom::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(this->get_logger(), "Heading Angle Node Initialized");
  }

private:
  void timer_callback() {
    // stop
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = message.linear.y = 0.0;
    message.angular.z = 0.0;
    RCLCPP_DEBUG(this->get_logger(), "Publishing: '%.2f' and '%.2f'",
                 message.linear.x, message.angular.z);
    pub_cmd_vel_->publish(message);
    // turn
    bool success_ = false;
    if (!success_)
      RCLCPP_DEBUG(this->get_logger(), "No success");
    success_ = this->closedLoopOdomTurn(true, 1.57079633);
  }
  bool closedLoopOdomTurn(bool clockwise, double radians);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};

// turn to a specified heading angle based on odometry information
bool ClosedLoopOdom::closedLoopOdomTurn(bool clockwise, double radians) {
  // convert angle value to range between -2 * M_PI and 2 * M_PI
  while (radians < 0)
    radians += 2 * M_PI;
  while (radians > 2 * M_PI)
    radians -= 2 * M_PI;

  // variables to get the initial and latest transform
  geometry_msgs::msg::TransformStamped initial_transform_msg;
  tf2::Transform initial_transform_tf2;
  geometry_msgs::msg::TransformStamped latest_transform_msg;
  tf2::Transform latest_transform_tf2;

  // try to record the initial transform from the odometry to the base frame
  try {
    initial_transform_msg = tf_buffer_->lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                "base_footprint", "odom", ex.what());
    return false;
  }

  tf2::impl::Converter<true, false>::convert(initial_transform_msg.transform,
                                             initial_transform_tf2);

  // initialize Twist message object
  auto base_cmd = geometry_msgs::msg::Twist();
  // command to turn (rad/sec)
  base_cmd.linear.x = base_cmd.linear.y = 0.0;
  base_cmd.angular.z = 0.1;
  if (clockwise)
    base_cmd.angular.z = -base_cmd.angular.z;

  // the axis we want to be rotating by
  tf2::Vector3 desired_turn_axis(0, 0, 1);
  if (!clockwise)
    desired_turn_axis = -desired_turn_axis;

  bool done = false;

  while (!done && rclcpp::ok()) {
    // publish to /cmd_vel topic to move the robot
    pub_cmd_vel_->publish(base_cmd);
    rclcpp::sleep_for(50000000ns);

    // get the latest transform
    try {
      latest_transform_msg = tf_buffer_->lookupTransform(
          "base_footprint", "odom", rclcpp::Time(0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "2. Could not transform %s to %s: %s",
                  "base_footprint", "odom", ex.what());
      break;
    }

    tf2::impl::Converter<true, false>::convert(latest_transform_msg.transform,
                                               latest_transform_tf2);

    tf2::Transform relative_transform =
        initial_transform_tf2.inverse() * latest_transform_tf2;
    tf2::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
    double angle_turned = relative_transform.getRotation().getAngle();
    if (fabs(angle_turned) < 1.0e-2)
      continue;

    if (actual_turn_axis.dot(desired_turn_axis) < 0)
      angle_turned = 2 * M_PI - angle_turned;

    if (angle_turned > radians)
      done = true;
  }
  if (done) {
    // stop the robot
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.0;
    pub_cmd_vel_->publish(base_cmd);
    return true;
  }

  return false;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopOdom>());
  rclcpp::shutdown();
  return 0;
}