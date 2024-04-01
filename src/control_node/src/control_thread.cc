
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "dji_motor.h"
#include "hal_can.h"
#include "mecanum.h"
#include "pid.h"

using namespace std::chrono_literals;
using namespace irobot_ec::hal;
using namespace irobot_ec::components::motor;
using irobot_ec::modules::algorithm::Mecanum;
using irobot_ec::modules::algorithm::PID;

extern std::shared_ptr<Can> chassis_can;
extern std::shared_ptr<Can> gimbal_can;

std::shared_ptr<GM6020> gimbal_pitch, gimbal_yaw;
std::shared_ptr<M3508> shoot_l, shoot_r;
std::shared_ptr<M3508> lf, rf, lb, rb;
std::shared_ptr<M2006> loader;
Mecanum chassis_solver(0.35f, 0.35f);

/**
 * @brief    底盘控制循环
 */
void ChassisLoop() {
  chassis_solver.Calculate(0.f, 1.f, 0.f);
  lf->SetCurrent(chassis_solver.v_lf() * 1000);
  rf->SetCurrent(-chassis_solver.v_rf() * 1000);
  lb->SetCurrent(chassis_solver.v_lb() * 1000);
  rb->SetCurrent(-chassis_solver.v_rb() * 1000);
  DjiMotorBase::SendCommand();
}

/**
 * @brief    云台控制循环
 */
void GimbalLoop() {}

/**
 * @brief    控制线程
 * @note     该线程负责底盘和云台的控制，控制逻辑在ChassisLoop和GimbalLoop两个函数里
 */
void ControlThread() {
  // 初始化电机对象
  gimbal_pitch = std::make_shared<GM6020>(*gimbal_can, 2);
  gimbal_yaw = std::make_shared<GM6020>(*gimbal_can, 1);
  shoot_l = std::make_shared<M3508>(*gimbal_can, 2);
  shoot_r = std::make_shared<M3508>(*gimbal_can, 1);
  loader = std::make_shared<M2006>(*gimbal_can, 3);
  lf = std::make_shared<M3508>(*chassis_can, 2);
  rf = std::make_shared<M3508>(*chassis_can, 1);
  lb = std::make_shared<M3508>(*chassis_can, 3);
  rb = std::make_shared<M3508>(*chassis_can, 4);

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(1ms);
    ChassisLoop();
    GimbalLoop();
  }
}