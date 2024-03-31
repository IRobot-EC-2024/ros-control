
#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

class ControlNode : public rclcpp::Node {
 public:
  ControlNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  // ---params---
  int param_hardware_settings_usb_vid_;
  int param_hardware_settings_usb_pid_;
  int param_hardware_settings_usb_read_endpoint_;
  int param_hardware_settings_usb_write_endpoint_;
  int param_hardware_settings_usb_read_timeout_;
  int param_hardware_settings_usb_write_timeout_;
};

#endif