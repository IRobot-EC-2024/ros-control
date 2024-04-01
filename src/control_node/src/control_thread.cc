
#include <chrono>
#include <iostream>
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

extern Can chassis_can;
extern Can gimbal_can;

GM6020 *gimbal_pitch, *gimbal_yaw;
M3508 *shoot_l, *shoot_r;
M3508 *lf, *rf, *lb, *rb;
M2006 *loader;
Mecanum chassis_solver(0.35f, 0.35f);

void ChassisLoop() {
  chassis_solver.Calculate(0.f, 0.f, -1.5f);
  lf->SetCurrent(-chassis_solver.v_lf() * 1000);
  rf->SetCurrent(chassis_solver.v_rf() * 1000);
  lb->SetCurrent(-chassis_solver.v_lb() * 1000);
  rb->SetCurrent(chassis_solver.v_rb() * 1000);
  DjiMotorBase::SendCommand();
}

void GimbalLoop() {}

void ControlThread() {
  gimbal_pitch = new GM6020(gimbal_can, 2);
  gimbal_yaw = new GM6020(gimbal_can, 1);
  shoot_l = new M3508(gimbal_can, 2);
  shoot_r = new M3508(gimbal_can, 1);
  loader = new M2006(gimbal_can, 3);
  lf = new M3508(chassis_can, 2);
  rf = new M3508(chassis_can, 1);
  lb = new M3508(chassis_can, 3);
  rb = new M3508(chassis_can, 4);

  while (rclcpp::ok()) {
    std::this_thread::sleep_for(1ms);
    ChassisLoop();
    GimbalLoop();
  }
}