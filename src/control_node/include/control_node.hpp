
#ifndef CONTROL_NODE_HPP
#define CONTROL_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

/**
 * @brief    控制节点
 * @note     这是这个程序唯一的节点
 * @note     这里没有控制逻辑，读取完参数之后运行交给各个线程处理
 */
class ControlNode : public rclcpp::Node {
 public:
  ControlNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  /** 以下都是在config/*.yaml中定义的参数 **/
  /** hardware_settings.yaml **/
  int param_hardware_settings_usb_vid_;
  int param_hardware_settings_usb_pid_;
  int param_hardware_settings_usb_read_endpoint_;
  int param_hardware_settings_usb_write_endpoint_;
  int param_hardware_settings_usb_read_timeout_;
  int param_hardware_settings_usb_write_timeout_;
  std::string param_hardware_settings_can_chassis_dev_;
  std::string param_hardware_settings_can_gimbal_dev_;

  std::thread can_polling_thread_;
  std::thread control_thread_;
};

#endif