
#ifndef EC_LIB_LINUX_HAL_USB_CDC_H
#define EC_LIB_LINUX_HAL_USB_CDC_H

#include <libusb-1.0/libusb.h>

namespace irobot_ec::hal {

class UsbcdcEndpoint {
 public:
  UsbcdcEndpoint(uint vid, uint pid, uint read_endpoint, uint write_endpoint, uint read_timeout, uint write_timeout);
  ~UsbcdcEndpoint() = default;
  void open();
  void close();
  int read(unsigned char* buffer, size_t len);
  int write(unsigned char* buffer, size_t len);

 private:
  uint vid_;
  uint pid_;
  uint read_endpont_;
  uint write_endpoint_;
  uint read_timeout_;
  uint write_timeout_;
  libusb_context* libusb_context_;
  libusb_device_handle* device_handle;
};

}  // namespace irobot_ec::hal

#endif  // EC_LIB_LINUX_HAL_USB_CDC_H
