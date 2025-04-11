#include "buraq_hw/wheel.h"

#include <cmath>

namespace buraq_hw {

void Wheel::Setup(const std::string& wheel_name, int ticks_per_rev) {
  name_ = wheel_name;
  rads_per_tick_ = (2 * M_PI) / ticks_per_rev;
}

double Wheel::Angle() { return enc_ * rads_per_tick_; }

}  // namespace buraq_hw