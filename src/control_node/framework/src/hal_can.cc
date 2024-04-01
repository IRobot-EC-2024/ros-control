
#include "hal_can.h"

#include <cstring>
#include <iostream>
#include <stdexcept>

namespace irobot_ec::hal {

/****************************/
// CLASS CanDeviceBase
/****************************/
CanDeviceBase::~CanDeviceBase() { this->hcan_->UnregisterDevice(*this); }

CanDeviceBase::CanDeviceBase(Can *hcan, uint32_t rx_std_id) : hcan_(hcan), rx_std_id_(rx_std_id) {
  this->hcan_->RegisterDevice(*this);
}

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

  while (write(this->socket_fd_, &this->tx_buf_, sizeof(this->tx_buf_)) == -1) {
  }
}

/**
 *  @brief    接收数据，根据id区分数据包，调用对应设备的回调函数
 */
void Can::Recv() noexcept {
  struct can_frame frame;
  int t = read(this->socket_fd_, &frame, sizeof(frame));
  if (t <= 0) {
    return;
  }

  auto receipient_device = this->device_list_.find(frame.can_id);
  if (receipient_device != this->device_list_.end()) {
    receipient_device->second->RxCallback(std::make_unique<struct can_frame>(frame));
  }

  // {
  //   // 将接收到的数据包放入消息队列
  //   std::lock_guard<std::mutex> lock(this->queue_synchronizer_);
  //   this->message_queue_.push_back(std::make_unique<struct can_frame>(frame));
  // }
}

/**
 * @brief     消息发布线程
 */
void Can::MessagePublishThread() {
  std::unique_ptr<struct can_frame> msg;
  while (true) {
    {
      std::lock_guard<std::mutex> lock(this->queue_synchronizer_);
      // 消息队列为空时，继续等待
      if (this->message_queue_.empty()) {
        continue;
      }
      // 消息队列数量超过50时，清空队列并打印错误信息
      if (this->message_queue_.size() > 50) {
        std::cerr << this->device_name_ << " message queue overflow" << std::endl;
        this->message_queue_.clear();
      }
      msg = std::move(this->message_queue_.front());
      this->message_queue_.pop_front();
    }

    // Can对象析构时，退出线程
    if (!this->is_opened_) {
      break;
    }
    // 根据报文ID找到对应的设备，调用回调函数
    auto receipient_device = this->device_list_.find(msg->can_id);
    if (receipient_device != this->device_list_.end()) {
      receipient_device->second->RxCallback(std::move(msg));
    }
  }
}

/**
 * @param dev can设备名, 例如"can0", "can1"，具体可以通过ifconfig命令查看
 */
Can::Can(const std::string &dev) : device_name_(dev), is_opened_(false) {
  this->socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (this->socket_fd_ < 0) {
    throw std::runtime_error(this->device_name_ + " open error");
  }

  // 配置 Socket CAN 为阻塞IO
  int flags = fcntl(this->socket_fd_, F_GETFL, 0);
  fcntl(this->socket_fd_, F_SETFL, flags | (~O_NONBLOCK));

  // 指定can设备
  strcpy(this->interface_request_.ifr_name, dev.c_str());

  ioctl(this->socket_fd_, SIOCGIFINDEX, &this->interface_request_);
  this->addr_.can_family = AF_CAN;
  this->addr_.can_ifindex = this->interface_request_.ifr_ifindex;

  // 配置过滤器，接收所有数据帧
  this->filter_.can_id = 0x0;
  this->filter_.can_mask = 0x0;
  setsockopt(this->socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &this->filter_, sizeof(this->filter_));

  // 将套接字与can设备绑定
  bind(this->socket_fd_, (struct sockaddr *)&this->addr_, sizeof(this->addr_));

  // 启动消息发布线程
  // this->message_publish_thread_ = std::thread(&Can::MessagePublishThread, this);

  this->is_opened_ = true;
}

Can::~Can() {
  close(this->socket_fd_);
  this->is_opened_ = false;
  // this->message_publish_thread_.join();
}

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

/**
 * @brief  返回消息队列中的消息数量
 * @return 消息数量
 */
uint Can::queued() { return this->message_queue_.size(); }

}  // namespace irobot_ec::hal
