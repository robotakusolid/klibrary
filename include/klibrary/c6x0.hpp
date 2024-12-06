#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "motor_base.hpp"
#include <tutrcos/module/c6x0.hpp>
#include <tutrcos/peripheral/can_base.hpp>
#include <tutrcos/utility.hpp>

namespace klibrary {
class C6x0 : public MotorBase {

public:
  C6x0(tutrcos::module::C6x0 &c6, Dir dir, float reduction_ratio,
       EncoderBase *enc = nullptr)
      : MotorBase{dir,
                  ((1.0f / utility::to_underlying(c6.get_type()) *
                    reduction_ratio)),
                  (enc == nullptr) ? c6.get_cpr() : enc->get_cpr(), enc},
        c6_{c6} {}

  bool update() override {
    control();
    return true;
  }

private:
  tutrcos::module::C6x0 &c6_;
};
} // namespace klibrary