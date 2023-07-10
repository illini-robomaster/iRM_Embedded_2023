#include "vfd.h"

namespace display {

VFD::VFD(bsp::GPIO* din, bsp::GPIO* clk, bsp::GPIO* cs, bsp::GPIO* rst, bsp::GPIO* en) {
  din_ = din;
  clk_ = clk;
  cs_ = cs;
  rst_ = rst;
  en_ = en;
}

}
