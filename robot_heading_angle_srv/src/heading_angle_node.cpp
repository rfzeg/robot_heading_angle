
/*
* Mobile Robot Closed Loop Odometry Controller
* Author: Roberto Zegers R.
* License: BSD-3-Clause
* Date: Sept, 2022
* Based on the tutorial: Using the base controller with odometry and transform information
* https://wiki.ros.org/pr2_controllers/Tutorials/
*/


#include <iostream>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

class ClosedLoopOdom {
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;

public:
  // ROS node initialization
  ClosedLoopOdom(ros::NodeHandle &nh) {
    nh_ = nh;
    // set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  // turn to a specified heading angle based on odometry information
  bool closedLoopOdomTurn(bool clockwise, double radians) {
    while (radians < 0)
      radians += 2 * M_PI;
    while (radians > 2 * M_PI)
      radians -= 2 * M_PI;

    // wait for the listener to get the first message
    listener_.waitForTransform("base_link", "odom", ros::Time(0),
                               ros::Duration(1.0));

    // variables to get the initial and current transform
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    // record the initial transform from the odometry to the base frame
    listener_.lookupTransform("base_link", "odom", ros::Time(0),
                              start_transform);

    // initialize Twist message object
    geometry_msgs::Twist base_cmd;
    // command to turn (rad/sec)
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.1;
    if (clockwise)
      base_cmd.angular.z = -base_cmd.angular.z;

    // the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0, 0, 1);
    if (!clockwise)
      desired_turn_axis = -desired_turn_axis;

    ros::Rate rate(10.0);
    bool done = false;
    while (!done && nh_.ok()) {
      // publish to /cmd_vel topic to move the robot
      cmd_vel_pub_.publish(base_cmd);
      rate.sleep();
      // get the current transform
      try {
        listener_.lookupTransform("base_link", "odom", ros::Time(0),
                                  current_transform);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        break;
      }
      tf::Transform relative_transform =
          start_transform.inverse() * current_transform;
      tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
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
      cmd_vel_pub_.publish(base_cmd);
      return true;
    }

    return false;
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometry_based_controller");
  ros::NodeHandle nh;

  ClosedLoopOdom odomController(nh);
  std::cout << "Press any key to start" << std::endl;
  std::cin.get();

  std::cout << "Turning Right pi/2 radians" << std::endl;
  odomController.closedLoopOdomTurn(true, 1.57079633);

  std::cout << "Done, press any key to exit" << std::endl;
  std::cin.get();
}
