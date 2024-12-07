#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "motor_base.hpp"
#include <tutrcos/module/sts3215.hpp>

namespace klibrary {

class STS3215 : public MotorBase {
public:
  STS3215(tutrcos::module::STS3215 &sts, Dir dir, float reduction_ratio,
          EncoderBase *enc = nullptr)
      : MotorBase{dir, reduction_ratio,
                  ((enc != nullptr) ? enc->get_cpr() : 4096), enc},
        sts_{sts} {}

  bool update() override {
    sts_.update();
    if (enc_ != nullptr) {
      set_count(enc_->get_count() + offset_);
    } else {
      set_count(sts_.get_count() + offset_);
    }
    control();
    return true;
  }

private:
  tutrcos::module::STS3215 &sts_;
};

} // namespace klibrary
