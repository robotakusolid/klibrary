#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "motor_base.hpp"
#include <tutrcos/module/c6x0.hpp>

namespace klibrary {
class C6x0 : public MotorBase {

public:
  C6x0(tutrcos::module::C6x0 &c6, Dir dir, float reduction_ratio,
       EncoderBase *enc = nullptr)
      : MotorBase{dir,
                  (((c6.get_type() == tutrcos::module::C6x0::Type::C610)
                        ? (1.0f / 36)
                    : (c6.get_type() == tutrcos::module::C6x0::Type::C620)
                        ? (1.0f / 19)
                        : 1) *
                   reduction_ratio),
                  ((enc != nullptr) ? enc->get_cpr() : c6.get_cpr()), enc},
        c6_{c6} {}

  bool update() override {
    if (enc_ != nullptr) {
      set_count(enc_->get_count() + offset_);
    } else {
      set_count(c6_.get_count() + offset_);
    }
    control();
    return true;
  }

private:
  tutrcos::module::C6x0 &c6_;
};
} // namespace klibrary