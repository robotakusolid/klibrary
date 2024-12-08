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
          EncoderBase *enc, float cut_point)
      : MotorBase{dir, reduction_ratio,
                  ((enc != nullptr) ? enc->get_cpr() : 4096), enc},
        sts_{sts}, cut_point_{static_cast<int64_t>(get_cpr() * cut_point)} {}

  STS3215(tutrcos::module::STS3215 &sts, Dir dir, float reduction_ratio,
          EncoderBase *enc)
      : STS3215{sts, dir, reduction_ratio, enc, 0.5} {}

  STS3215(tutrcos::module::STS3215 &sts, Dir dir, float reduction_ratio,
          float cut_point = 0.5)
      : STS3215{sts, dir, reduction_ratio, nullptr, cut_point} {}

  bool update() override {
    int64_t count = 0;
    RETURNFALSE(sts_.update());
    if (enc_ != nullptr) {
      RETURNFALSE(enc_->update());
      count = enc_->get_count();
    } else {
      count = sts_.get_count();
    }
    count += offset_;
    if (sts_.get_mode() == tutrcos::module::STS3215::Mode::SINGLE_TURN) {
      if (count > cut_point_) {
        count -= get_cpr();
      }
    }
    set_count(count);
    control();
    return true;
  }

  void set_input(float value) override { ; }
  void stop() override { ; }

private:
  tutrcos::module::STS3215 &sts_;
  const int64_t cut_point_;
};

} // namespace klibrary
