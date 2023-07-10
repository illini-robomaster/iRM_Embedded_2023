#pragma once

#include "bsp_gpio.h"

namespace display {

class VFD {
 public:
  VFD(bsp::GPIO* din, bsp::GPIO* clk, bsp::GPIO* cs, bsp::GPIO* rst, bsp::GPIO* en);
 private:
  bsp::GPIO* din_;
  bsp::GPIO* clk_;
  bsp::GPIO* cs_;
  bsp::GPIO* rst_;
  bsp::GPIO* en_;
};

}
