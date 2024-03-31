
#include "hal_can.h"

#include <cstring>
#include <stdexcept>

constexpr const char *GIMBAL_CAN = "can1";
constexpr uint16_t AMMOR = 0x201;
constexpr uint16_t AMMOL = 0x202;
constexpr uint16_t ROTOR = 0x203;
constexpr uint16_t YAW = 0x205;
constexpr uint16_t PITCH = 0x206;
constexpr uint16_t GIMBAL_COMMAND = 0x1FF;
constexpr uint16_t SHOOT_COMMAND = 0x200;

constexpr const char *CHASSIS_CAN = "can0";
constexpr uint16_t WHEEL1 = 0x201;
constexpr uint16_t WHEEL2 = 0x202;
constexpr uint16_t WHEEL3 = 0x203;
constexpr uint16_t WHEEL4 = 0x204;
constexpr uint16_t CHASSIS_COMMAND = 0x200;

namespace irobot_ec::hal {

/****************************/
// CLASS CanDeviceBase
/****************************/
CanDeviceBase::~CanDeviceBase() { this->hcan_->UnregisterDevice(*this); }

CanDeviceBase::CanDeviceBase(Can *hcan, uint32_t rx_std_id) { this->hcan_->RegisterDevice(*this); }

Can &CanDeviceBase::GetCanBus() { return *this->hcan_; }

void CanDeviceBase::SetTxStdId(uint32_t id) { tx_std_id_ = id; }

void CanDeviceBase::Transmit(u_char *buf, u_char dlc) { this->hcan_->Send(this->tx_std_id_, buf, dlc); }

/****************************/
// CLASS Can
/****************************/
/**
 * @brief   发送数据
 * @param   id  报文的标准帧ID
 * @param   buf 数据指针
 * @param   dlc 数据长度
 */
void Can::Send(uint id, u_char *buf, u_char dlc) {
  if (dlc > 8) {
    throw std::invalid_argument("dlc must be less than 8");
  }

  this->tx_buf_.can_id = id;
  this->tx_buf_.can_dlc = dlc;

  for (int i = 0; i < (int)dlc; i++) this->tx_buf_.data[i] = buf[i];

  int t = write(this->socket_fd_, &this->tx_buf_, sizeof(this->tx_buf_));
  if (t >= 0) {
    throw std::runtime_error("can send error");
  }
}

/**
 *  @brief    接收数据，根据id区分数据包，调用对应设备的回调函数
 */
void Can::Recv() {
  struct can_frame frame;
  int t = write(this->socket_fd_, &frame, sizeof(frame));
  if (t <= 0) {
    throw std::runtime_error("can receive error");
  }

  auto receipient_device = this->device_list_.find(frame.can_id);

  if (receipient_device != this->device_list_.end()) {
    receipient_device->second->RxCallback(std::make_unique<struct can_frame>(frame));
  }
}

/**
 * @param dev can设备名, 例如"can0", "can1"，具体可以通过ifconfig命令查看
 */
Can::Can(const std::string &dev) {
  this->socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (this->socket_fd_ < 0) {
    throw std::runtime_error("can socket init error");
  }

  // 配置 Socket CAN 为阻塞IO
  int flags = fcntl(this->socket_fd_, F_GETFL, 0);
  fcntl(this->socket_fd_, F_SETFL, flags | (~O_NONBLOCK));

  // 指定can设备
  strcpy(this->interface_request_.ifr_name, dev.c_str());

  ioctl(this->socket_fd_, SIOCGIFINDEX, &this->interface_request_);
  this->addr_.can_family = AF_CAN;
  this->addr_.can_ifindex = this->interface_request_.ifr_ifindex;

  // 将套接字与can设备绑定
  bind(this->socket_fd_, (struct sockaddr *)&this->addr_, sizeof(this->addr_));
}

Can::~Can() { close(this->socket_fd_); }

/**
 * @brief 向这个Can总线对象上注册一个设备
 * @param device 设备引用
 */
void Can::RegisterDevice(CanDeviceBase &device) { this->device_list_.emplace(device.rx_std_id_, &device); }

/**
 * @brief 从这个Can总线对象上注销一个设备
 * @param device 设备引用
 */
void Can::UnregisterDevice(CanDeviceBase &device) {
  auto dev = this->device_list_.find(device.rx_std_id_);
  if (dev != this->device_list_.end()) {
    this->device_list_.erase(dev);
  }
}

}  // namespace irobot_ec::hal