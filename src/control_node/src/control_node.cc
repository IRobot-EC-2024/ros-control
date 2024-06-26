
#include "control_node.hpp"

#include <pthread.h>

#include <chrono>

#define info(args...) RCLCPP_INFO(this->get_logger(), args)

using namespace std::chrono_literals;

// 这里声明了所有线程
void ControlThread();
void CanPollingThread(std::string &chassis_can_dev, std::string &gimbal_can_dev);
void UsbCdcPollingThread();

ControlNode::ControlNode(const rclcpp::NodeOptions &options) : rclcpp::Node("control_node", options) {
  /** hardware_settings.yaml **/
  this->declare_parameter<std::string>("can_chassis_dev");
  this->declare_parameter<std::string>("can_gimbal_dev");
  this->declare_parameter<int>("usb_vid");
  this->declare_parameter<int>("usb_pid");
  this->declare_parameter<int>("usb_read_endpoint");
  this->declare_parameter<int>("usb_write_endpoint");
  this->declare_parameter<int>("usb_read_timeout");
  this->declare_parameter<int>("usb_write_timeout");
  this->get_parameter<std::string>("can_chassis_dev", this->param_hardware_settings_can_chassis_dev_);
  this->get_parameter<std::string>("can_gimbal_dev", this->param_hardware_settings_can_gimbal_dev_);
  this->get_parameter<int>("usb_vid", this->param_hardware_settings_usb_vid_);
  this->get_parameter<int>("usb_pid", this->param_hardware_settings_usb_pid_);
  this->get_parameter<int>("usb_read_endpoint", this->param_hardware_settings_usb_read_endpoint_);
  this->get_parameter<int>("usb_write_endpoint", this->param_hardware_settings_usb_write_endpoint_);
  this->get_parameter<int>("usb_read_timeout", this->param_hardware_settings_usb_read_timeout_);
  this->get_parameter<int>("usb_write_timeout", this->param_hardware_settings_usb_write_timeout_);
  /****************************/

  /** 启动线程 **/
  this->can_polling_thread_ = std::thread(CanPollingThread, std::ref(this->param_hardware_settings_can_chassis_dev_),
                                          std::ref(this->param_hardware_settings_can_gimbal_dev_));
  std::this_thread::sleep_for(50ms);  // 等待上一个线程初始化
  this->usbcdc_polling_thread_ = std::thread(UsbCdcPollingThread);
  std::this_thread::sleep_for(50ms);  // 等待上一个线程初始化
  this->control_thread_ = std::thread(ControlThread);
}

/**
 * @brief    杀死所有线程
 * @warning  只在退出的时候调用这个函数！！
 */
void ControlNode::KillThreads() {
  pthread_cancel(this->can_polling_thread_.native_handle());
  pthread_cancel(this->control_thread_.native_handle());
  pthread_cancel(this->usbcdc_polling_thread_.native_handle());
  this->can_polling_thread_.join();
  this->control_thread_.join();
  this->usbcdc_polling_thread_.join();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto control_node = std::make_shared<ControlNode>();
  rclcpp::spin(control_node);
  control_node->KillThreads();
  rclcpp::shutdown();
  return 0;
}