#include "vesc_ackermann/vesc_to_odom.hpp"
#include <cmath>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>

namespace vesc_ackermann
{

using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std_msgs::msg::Float64;

VescToOdom::VescToOdom(const rclcpp::NodeOptions & options)
    : Node("vesc_to_odom_node", options),
      odom_frame_("odom"),
      base_frame_("base_link"),
      publish_tf_(true), // Always publish TF
      x_(0.0),
      y_(0.0),
      wheelbase_(0.3),
      theta_(0.0), //initial heading angle
      motor_speed_(0.0), // Initialize speed
      servo_angle_(0.0) // Initialize steering angle
{
  // get ROS parameters
  odom_frame_ = declare_parameter("odom_frame", odom_frame_);
  base_frame_ = declare_parameter("base_frame", base_frame_);
  publish_tf_ = declare_parameter("publish_tf", publish_tf_);

  // create odom publisher
  odom_pub_ = create_publisher<Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  // Subscribe to the motor_speed topic
  motor_speed_subscriber_ = this->create_subscription<Float64>(
      "motor_speed", 10, std::bind(&VescToOdom::motorSpeedCallback, this, _1));

  // Subscribe to the servo_angle topic
  servo_angle_subscriber_ = this->create_subscription<Float64>(
      "servo_angle", 10, std::bind(&VescToOdom::servoAngleCallback, this, _1));

  // Set the update rate to 10 Hz (0.1 seconds)
  update_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&VescToOdom::updateCallback, this));
}

//Define the map function
double map(double value, double in_min, double in_max, double out_min, double out_max) {
    double mappedValue = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    return mappedValue;
}

void VescToOdom::motorSpeedCallback(const Float64::SharedPtr msg)
{
  // Process received motor speed message
  double read_speed_ = msg->data;
  motor_speed_ = map(read_speed_, -2500.0, 2500.0, -3.0, 3.0);
}

void VescToOdom::servoAngleCallback(const Float64::SharedPtr msg)
{
  // Process received servo angle message
  double read_angle_ = msg->data;
  servo_angle_ =  map(read_angle_, 65.0, 67.0, -40.0/180.0*M_PI, 40.0/180.0*M_PI);
}

void VescToOdom::updateCallback()
{
  // This function is called every 0.1 seconds, update your odometry here
  
  // Propagate odometry
  double x_dot = motor_speed_ * cos(theta_);
  double y_dot = motor_speed_ * sin(theta_);
  double theta_dot = motor_speed_ / wheelbase_ * tan(servo_angle_) ;
  double dt = 0.1; // Set sampling time to 0.1 sec
  theta_ += theta_dot * dt;
  x_ += x_dot * dt;
  y_ += y_dot * dt;

  // Publish odometry message
  Odometry odom;
  odom.header.frame_id = odom_frame_;
  odom.header.stamp = now();
  odom.child_frame_id = base_frame_;

  // Position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  //For a rotation purely around the z-axis (common in 2D navigation)
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(theta_ / 2.0);
  odom.pose.pose.orientation.w = cos(theta_ / 2.0);

  // Velocity ("in the coordinate frame given by the child_frame_id")
  odom.twist.twist.linear.x = motor_speed_;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = 0.0; // Should be current_angular_velocity;

  if (publish_tf_) {
    TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }

  if (rclcpp::ok()) {
    odom_pub_->publish(odom);
  }
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::VescToOdom)
