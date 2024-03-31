
#include "hal_usbcdc.h"

#include <stdexcept>

namespace irobot_ec::hal {

UsbcdcEndpoint::UsbcdcEndpoint(uint vid, uint pid, uint read_endpoint, uint write_endpoint, uint read_timeout,
                               uint write_timeout)
    : vid_(vid),
      pid_(pid),
      read_endpont_(read_endpoint),
      write_endpoint_(write_endpoint),
      read_timeout_(read_timeout),
      write_timeout_(write_timeout),
      libusb_context_(NULL) {}

void UsbcdcEndpoint::open() {
  // 初使化libusb
  int ret = libusb_init(&libusb_context_);
  if (ret != libusb_error::LIBUSB_SUCCESS) {
    libusb_exit(libusb_context_);
    throw std::runtime_error("libusb init failed");
  }
  // 打开指定厂商的某类产品
  device_handle = libusb_open_device_with_vid_pid(libusb_context_, vid_, pid_);
  if (device_handle == NULL) {
    libusb_exit(libusb_context_);
    throw std::runtime_error("libusb open device failed");
  }

  // 卸载使用的USBCDC接口内核驱动
  while (libusb_kernel_driver_active(device_handle, 0) != libusb_error::LIBUSB_SUCCESS) {
    libusb_detach_kernel_driver(device_handle, 0);
  }

  // 打开指定接口
  if ((libusb_claim_interface(device_handle, 0)) != libusb_error::LIBUSB_SUCCESS) {
    libusb_close(device_handle);
    libusb_exit(libusb_context_);
    throw std::runtime_error("libusb claim interface failed");
  }
}

void UsbcdcEndpoint::close() {
  libusb_close(device_handle);
  libusb_exit(libusb_context_);
}

int UsbcdcEndpoint::read(unsigned char* buffer, size_t len) {
  int ReceiveSize = 0;
  if (libusb_bulk_transfer(device_handle, read_endpont_, buffer, len, &ReceiveSize, read_timeout_) < 0) {
    return -1;
  }
  return ReceiveSize;
}

int UsbcdcEndpoint::write(unsigned char* buffer, size_t len) {
  int ActualTransmitSize = 0;
  return libusb_bulk_transfer(device_handle, write_endpoint_, buffer, len, &ActualTransmitSize, write_timeout_);
}

}  // namespace irobot_ec::hal