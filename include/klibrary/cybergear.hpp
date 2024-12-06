#pragma once

#include "motor_base.hpp"
#include <tutrcos/module/cybergear.hpp>
#include <tutrcos/peripheral/can_base.hpp>
#include <tutrcos/utility.hpp>

namespace klibrary {
class Cybergear : public MotorBase {
public:
  Cybergear(tutrcos::module::Cybergear &cyber, Dir dir, float reduction_ratio,
            EncoderBase *enc)
      : MotorBase{dir, (reduction_ratio * ((enc == nullptr) ? (1 / 8) : 1)),
                  ((enc == nullptr) ? 1 : enc->get_cpr()), enc},
        cyber_{cyber} {}

  bool update() override {
    control();
    return true;
  }

private:
  tutrcos::module::Cybergear &cyber_;
};
} // namespace klibrary