#pragma once

#include "rotater_base.hpp"
#include <tutrcos/module/cybergear.hpp>

namespace klibrary {
class Cybergear : public RotaterBase {
public:
  using RB = RotaterBase;
  Cybergear(RB::Type rtype, tutrcos::module::Cybergear *cyber,
            RB::Dir motor_dir, EncoderBase *enc, RB::Dir enc_dir,
            float reduction_ratio = 1, RB::Turn tmode = RB::Turn::MULTI,
            float cut_point = 0.5)
      : RotaterBase{rtype,   cyber,           motor_dir, enc,
                    enc_dir, reduction_ratio, tmode,     cut_point} {
    cyber_ = cyber;
  }

  void set_input(float ampare) { cyber_->set_current(ampare); }
  void stop() override { ; }

private:
  tutrcos::module::Cybergear *cyber_;
};
} // namespace klibrary