
#ifndef EC_LIB_LINUX_HAL_CAN_H
#define EC_LIB_LINUX_HAL_CAN_H

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "thread_pool.hpp"

namespace irobot_ec::hal {

class CanDeviceBase;
class Can;

/**
 * @brief    CAN设备的基类
 */
class CanDeviceBase {
  friend class Can;

 public:
  ~CanDeviceBase();
  CanDeviceBase(Can *hcan, uint32_t rx_std_id);

  // no copy
  CanDeviceBase(const CanDeviceBase &) = delete;
  CanDeviceBase &operator=(const CanDeviceBase &) = delete;

  Can &GetCanBus();
  void SetTxStdId(uint32_t id);
  void Transmit(u_char *buf, u_char dlc);

 protected:
  virtual void RxCallback(std::unique_ptr<struct can_frame> msg) = 0;

  uint32_t tx_std_id_;  // tx standard id
  uint32_t rx_std_id_;  // rx standard id
  Can *hcan_;
};

/**
 * @brief    CAN通信包装类
 */
class Can {
  friend class CanDeviceBase;

 public:
  Can() = delete;
  Can(const std::string &dev);
  ~Can();

  // no copy
  Can(const Can &) = delete;
  Can &operator=(const Can &) = delete;

  void Recv();
  void Send(uint id, u_char *buf, u_char dlc);

 private:
  void RegisterDevice(CanDeviceBase &device);
  void UnregisterDevice(CanDeviceBase &device);

  std::unordered_map<uint32_t, CanDeviceBase *> device_list_;

  int socket_fd_;
  struct sockaddr_can addr_;
  struct ifreq interface_request_;

  struct can_frame tx_buf_;
  int can_id_;

  modules::ThreadPool thread_pool_{16};
};

}  // namespace irobot_ec::hal

#endif  // EC_LIB_LINUX_HAL_CAN_H
