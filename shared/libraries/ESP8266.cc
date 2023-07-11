#include "ESP8266.h"
#include <cstring>

namespace wifi {

ESP8266::ESP8266(bsp::UART* uart) {
  uart_ = uart;
}

bool ESP8266::OK() {
  char ret_cmd[] = "OK";
  uint8_t* data;

  uart_->Read(&data);
  return strstr((char*)data, ret_cmd) != nullptr;
}

void ESP8266::IsReady() {
  char check_cmd[] = "AT\r\n";
  uart_->Write((uint8_t*)check_cmd, strlen(check_cmd));
}

void ESP8266::SetMode() {
  char mode_cmd[] = "AT+CWMODE=3\r\n";
  uart_->Write((uint8_t*)mode_cmd, strlen(mode_cmd));
}

void ESP8266::Reset() {
  char reset_cmd[] = "AT+RST\r\n";
  uart_->Write((uint8_t*)reset_cmd, strlen(reset_cmd));
}

void ESP8266::ConnectWiFi() {
  char connect_cmd[] = "AT+CWJAP=\"AlcheicRonin\",\"88888888\"\r\n";
  uart_->Write((uint8_t*)connect_cmd, strlen(connect_cmd));
}

void ESP8266::SetConnectionType() {
  char mux_cmd[] = "AT+CIPMUX=0\r\n";
  uart_->Write((uint8_t*)mux_cmd, strlen(mux_cmd));
}

}
