#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "rotater_base.hpp"
#include <tutrcos/module/sts3215.hpp>

namespace klibrary {

class STS3215 : public RotaterBase {
public:
  using RB = RotaterBase;
  STS3215(RB::Type rtype, tutrcos::module::STS3215 *sts, RB::Dir motor_dir,
          EncoderBase *enc, RB::Dir enc_dir, float reduction_ratio = 1,
          RB::Turn tmode = RB::Turn::MULTI, float cut_point = 0.5)
      : RotaterBase{rtype,           sts,   motor_dir, enc, enc_dir,
                    reduction_ratio, tmode, cut_point} {
    sts_ = sts;
  }
  STS3215(RB::Type rtype, tutrcos::module::STS3215 *sts, RB::Dir motor_dir,
          float reduction_ratio = 1, RB::Turn tmode = RB::Turn::MULTI,
          float cut_point = 0.5)
      : STS3215{rtype,           sts,   motor_dir, nullptr, RB::Dir::FORWARD,
                reduction_ratio, tmode, cut_point} {}

  void set_raw_input(float value) override { sts_->set_input(value); }
  bool transmit() override { return sts_->transmit(); }
  void stop() override { ; }

private:
  tutrcos::module::STS3215 *sts_;
};

} // namespace klibrary
