#pragma once

#include "rotater_base.hpp"
#include <tutrcos/module/cybergear.hpp>

namespace klibrary {
class Cybergear : public RotaterBase {
public:
  using RB = RotaterBase;
  Cybergear(RB::Type rtype, tutrcos::module::Cybergear *cyber, RB::Dir motor_dir, EncoderBase *enc, RB::Dir enc_dir,
            float reduction_ratio = 1, RB::Turn tmode = RB::Turn::MULTI, float cut_point = 0.5)
      : RotaterBase{rtype, cyber,    motor_dir, enc, enc_dir, (((rtype == Type::MOTOR) ? (1 / 7.75f) : 1) * reduction_ratio),
                    tmode, cut_point} {
    cyber_ = cyber;
  }

  void set_raw_input(float ampare) override { cyber_->set_current(ampare); }
  bool transmit() override { return cyber_->transmit(); }
  void stop() override { cyber_->stop(); }
  void reset() override {
    cyber_->reset();
    cyber_->set_motor_mode(tutrcos::module::Cybergear::MotorMode::CURRENT);
    cyber_->enable();
  }

private:
  tutrcos::module::Cybergear *cyber_;
};
} // namespace klibrary