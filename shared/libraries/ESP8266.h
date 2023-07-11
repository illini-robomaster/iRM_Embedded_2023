#pragma once

#include "main.h"
#include "bsp_uart.h"

namespace wifi {

class ESP8266 {
 public:
  ESP8266(bsp::UART* uart);

  bool OK();
  void IsReady();
  void SetMode();
  void Reset();
  void ConnectWiFi();
  void SetConnectionType();
 private:
  bsp::UART* uart_;
};

}
