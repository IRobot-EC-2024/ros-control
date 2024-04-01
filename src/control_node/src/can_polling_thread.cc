
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#include "hal_can.h"

using namespace std::chrono_literals;
using namespace irobot_ec::hal;

std::shared_ptr<Can> chassis_can;
std::shared_ptr<Can> gimbal_can;

/**
 * @brief    CAN轮询接收线程
 */
void CanPollingThread(std::string &chassis_can_dev, std::string &gimbal_can_dev) {
  chassis_can = std::make_shared<Can>(chassis_can_dev);
  gimbal_can = std::make_shared<Can>(gimbal_can_dev);
  while (rclcpp::ok()) {
    chassis_can->Recv();
    gimbal_can->Recv();
  }
}