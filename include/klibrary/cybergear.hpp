#pragma once

#include "motor_base.hpp"
#include <tutrcos/module/cybergear.hpp>

namespace klibrary {
class Cybergear : public MotorBase {
public:
  Cybergear(tutrcos::module::Cybergear &cyber, Dir dir, float reduction_ratio,
            EncoderBase *enc)
      : MotorBase{dir, (reduction_ratio * ((enc != nullptr) ? 1 : (1.0f / 8))),
                  ((enc != nullptr) ? enc->get_cpr() : 1), enc},
        cyber_{cyber} {}

  bool update() override {
    if (enc_ != nullptr) {
      RETURNFALSE(enc_->update());
      set_count(enc_->get_count() + offset_);
    } else {
      // set_count(cyber_.get_count() + offset_);
    }
    control();
    return true;
  }

  void set_input(float ampare) { cyber_.set_current(ampare); }
  void stop() override { ; }

private:
  tutrcos::module::Cybergear &cyber_;
};
} // namespace klibrary