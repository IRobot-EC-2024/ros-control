

#ifndef EC_LIB_COMPONENTS_REMOTE_CONTROL_H
#define EC_LIB_COMPONENTS_REMOTE_CONTROL_H

#include "hal_usbcdc.h"

namespace irobot_ec::components::remote_control {

enum class SwitchState {
  kDown = 0,
  kMid = 1,
  kUp = 2,
};

typedef struct SbusFrame {
  float channal_rx;
  float channal_ry;
  float channal_lx;
  float channal_ly;
  float channal_dial;
  float mouse_x;
  float mouse_y;
  float mouse_z;

  bool switch_rdown;
  bool switch_rmid;
  bool switch_rup;
  bool switch_ldown;
  bool switch_lmid;
  bool switch_lup;
} SbusFrame;

class SbusRemote {
 public:
  void update(unsigned char *buffer);
  void process();

 private:
  unsigned char sbus_buff_[18];
  SbusFrame sbus_frame_;
};

}  // namespace irobot_ec::components::remote_control

#endif  // EC_LIB_COMPONENTS_REMOTE_CONTROL_H

/* EOF */
