#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "motor_base.hpp"
#include <tutrcos/module/sts3215.hpp>
#include <tutrcos/utility.hpp>

namespace klibrary {

class STS3215 : public MotorBase {
public:
  STS3215(tutrcos::module::STS3215 &sts, Dir dir, float reduction_ratio,
          EncoderBase *enc = nullptr)
      : MotorBase{dir, reduction_ratio,
                  (enc == nullptr) ? 4096 : enc->get_cpr(), enc},
        sts_{sts} {}

  bool update() override {
    control();
    sts_.update();
    set_count(sts_.get_count());
    return true;
  }

private:
  tutrcos::module::STS3215 &sts_;
};

} // namespace klibrary
