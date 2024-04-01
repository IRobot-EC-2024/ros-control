
#include "control_node.hpp"

#include <map>

#define info(args...) RCLCPP_INFO(this->get_logger(), args)

void ControlThread();
void CanPollingThread();

ControlNode::ControlNode(const rclcpp::NodeOptions &options) : rclcpp::Node("control_node", options) {
  /** hardware_settings.yaml **/
  this->declare_parameter<int>("usb_vid");
  this->declare_parameter<int>("usb_pid");
  this->declare_parameter<int>("usb_read_endpoint");
  this->declare_parameter<int>("usb_write_endpoint");
  this->declare_parameter<int>("usb_read_timeout");
  this->declare_parameter<int>("usb_write_timeout");
  this->get_parameter<int>("usb_vid", this->param_hardware_settings_usb_vid_);
  this->get_parameter<int>("usb_pid", this->param_hardware_settings_usb_pid_);
  this->get_parameter<int>("usb_read_endpoint", this->param_hardware_settings_usb_read_endpoint_);
  this->get_parameter<int>("usb_write_endpoint", this->param_hardware_settings_usb_write_endpoint_);
  this->get_parameter<int>("usb_read_timeout", this->param_hardware_settings_usb_read_timeout_);
  this->get_parameter<int>("usb_write_timeout", this->param_hardware_settings_usb_write_timeout_);

  this->can_polling_thread_ = std::thread(CanPollingThread);
  this->control_thread_ = std::thread(ControlThread);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto control_node = std::make_shared<ControlNode>();
  rclcpp::spin(control_node);
  rclcpp::shutdown();
  return 0;
}