
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "hal_can.h"

using namespace std::chrono_literals;
using namespace irobot_ec::hal;

Can chassis_can("can0");
Can gimbal_can("can1");

void CanPollingThread() {
  while (true) {
    chassis_can.Recv();
    gimbal_can.Recv();
  }
}