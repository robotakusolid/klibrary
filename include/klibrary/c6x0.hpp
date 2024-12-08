#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rotater_base.hpp"
#include <tutrcos/module/c6x0.hpp>

namespace klibrary {
class C6x0 : public RotaterBase {
public:
  using RB = RotaterBase;
  C6x0(RB::Type rtype, tutrcos::module::C6x0 *c6, RB::Dir motor_dir,
       EncoderBase *enc, RB::Dir enc_dir, float reduction_ratio = 1,
       RB::Turn tmode = RB::Turn::MULTI, float cut_point = 0.5)
      : RotaterBase{rtype,
                    c6,
                    motor_dir,
                    enc,
                    enc_dir,
                    (((rtype == Type::ENCODER) ? 1
                      : (c6->get_type() == tutrcos::module::C6x0::Type::C610)
                          ? (1.0f / 36)
                      : (c6->get_type() == tutrcos::module::C6x0::Type::C620)
                          ? (1.0f / 19)
                          : 1) *
                     reduction_ratio),
                    tmode,
                    cut_point} {
    c6_ = c6;
  }

  void set_input(float ampare) override { c6_->set_current(ampare * 1000); }
  void stop() override { ; }

private:
  tutrcos::module::C6x0 *c6_;
};
} // namespace klibrary