#pragma once

#include "main.h"

#include "utility.hpp"
#include <cmath>
#include <tutrcos.hpp>
#include <tutrcos/module/encoder_base.hpp>

using namespace tutrcos;
using namespace tutrcos::module;

namespace klibrary {
class RotaterBase : public EncoderBase {
#define FLT_MAX std::numeric_limits<float>::max()
#define RETURNFALSE(x)                                                         \
  {                                                                            \
    if (!x) {                                                                  \
      return false;                                                            \
    }                                                                          \
  }

#define NOTNULL(pointer, do_exist, do_null)                                    \
  {                                                                            \
    if (pointer != nullptr) {                                                  \
      do_exist                                                                 \
    } else {                                                                   \
      do_null                                                                  \
    }                                                                          \
  }

public:
  enum class Type { // どちらから回転角を取得するか
    MOTOR,
    ENCODER,
  };
  enum class Dir : int8_t {
    FORWARD = 1,
    REVERSE = -1,
  };
  enum class Turn {
    SINGLE,
    MULTI,
  };
  enum class Control {
    PI_D_RAD,
    PI_D_RADPS,
    RAW,
    ZERO,
  };

  typedef struct {
    Type rtype;
    EncoderBase *motor;
    Dir motor_dir;
    EncoderBase *enc;
    Dir enc_dir;
    float reduction_ratio = 1;
    Turn tmode = Turn::MULTI;
    float cut_point = 0.5;
  } InitStruct;

  typedef struct {
    Type rtype;
    EncoderBase *tmp;
    Dir dir;
    float reduction_ratio = 1;
    Turn tmode = Turn::MULTI;
    float cut_point = 0.5;
  } HalfInitStruct;

  RotaterBase(Type rtype, EncoderBase *motor, Dir motor_dir, EncoderBase *enc,
              Dir enc_dir, float reduction_ratio = 1, Turn tmode = Turn::MULTI,
              float cut_point = 0.5)
      : EncoderBase{(rtype == Type::MOTOR) ? motor->get_cpr() : enc->get_cpr()},
        i_dir_{utility::to_underlying(motor_dir)},
        o_dir_{utility::to_underlying((rtype == Type::ENCODER) ? enc_dir
                                                               : motor_dir)},
        reduction_ratio_{reduction_ratio}, turn_mode_{tmode},
        rotater_type_{rtype},
        cut_point_{static_cast<int64_t>(this->get_cpr() * cut_point)} {
    motor_ = motor;
    enc_ = enc;
  }

  RotaterBase(Type rtype, EncoderBase *tmp, Dir dir, float reduction_ratio = 1,
              Turn tmode = Turn::MULTI, float cut_point = 0.5)
      : RotaterBase{rtype, ((rtype == Type::MOTOR) ? tmp : nullptr),
                    dir,   ((rtype == Type::ENCODER) ? tmp : nullptr),
                    dir,   reduction_ratio,
                    tmode, cut_point} {}

  // RotaterBase(InitStruct init)
  //     : RotaterBase{init.rtype, init.motor,    init.motor_dir,
  //                   init.enc,   init.enc_dir,  init.reduction_ratio,
  //                   init.tmode, init.cut_point} {}

  // virtual bool update_motor() = 0;   //{ return control(); };
  // virtual void stop() = 0;           //{ set_input(0); }
  // virtual void set_input(float) = 0; //{ input_ = value; };
  virtual void stop() { set_input(0); }
  virtual void set_input(float value) { input_ = value; };

  bool update() override {
    int64_t count = 0;
    NOTNULL(motor_, {RETURNFALSE(motor_->update())}, ;)
    NOTNULL(enc_, {RETURNFALSE(enc_->update())}, ;)
    switch (rotater_type_) {
    case Type::MOTOR: {
      NOTNULL(motor_, { count = motor_->get_count(); }, ;)
      break;
    }
    case Type::ENCODER: {
      NOTNULL(enc_, { count = enc_->get_count(); }, ;)
      break;
    }
    }

    switch (turn_mode_) {
    case Turn::SINGLE: {
      if (count > cut_point_) {
        count -= this->get_cpr();
      }
      break;
    }
    case Turn::MULTI: {
      // int16_t delta = count - prev_count_;
      // if (delta > (get_cpr() / 2)) {
      //   delta -= get_cpr();
      // } else if (delta < -(get_cpr() / 2)) {
      //   delta += get_cpr();
      // }
      // set_count(get_count() + delta);
      break;
    }
    }
    // count_ = count;
    set_count(count);
    control();
    return true;
  }

  float get_rotation() override {
    return (static_cast<float>(get_count()) / this->get_cpr() *
                reduction_ratio_ -
            offset_rotation_) *
           o_dir_;
  }

  float get_rps() override {
    float rps = 0;
    switch (rotater_type_) {
    case Type::MOTOR: {
      NOTNULL(motor_, { rps = motor_->get_rps(); }, ;)
      break;
    }
    case Type::ENCODER: {
      NOTNULL(enc_, { rps = enc_->get_rps(); }, ;)
      break;
    }
    }
    return rps * o_dir_;
  }

  void set_control_mode(Control mode) { mode_ = mode; }

  Turn get_turn_mode() { return turn_mode_; }

  void set_offset_degree(float value) {
    // offset_ = value / 360. / reduction_ratio_ * get_cpr() * o_dir_;
    offset_rotation_ = value / 360.;
  }

  void set_ref(float ref) {
    uint32_t dt = tutrcos::core::Kernel::get_ticks() - prev_ticks_ref_;
    ref = pre_ref_ + std::clamp<float>(ref - pre_ref_, -ref_vel_limit_ * dt,
                                       ref_vel_limit_ * dt);
    ref_ = std::clamp<float>(ref, ref_limit_min_, ref_limit_max_);
    pre_ref_ = ref_;
    prev_ticks_ref_ += dt;
  }

  void set_ref_limit_rad(float ref_min, float ref_max, // min, max, /s
                         float ref_vel = FLT_MAX) {
    ref_limit_min_ = ref_min;
    ref_limit_max_ = ref_max;
    ref_vel_limit_ = abs(ref_vel);
  }

  void set_ref_limit_degree(float ref_min, float ref_max, // min, max, /s
                            float ref_vel = FLT_MAX) {
    ref_limit_min_ = deg2rad(ref_min);
    ref_limit_max_ = deg2rad(ref_max);
    ref_vel_limit_ = deg2rad(abs(ref_vel));
  }

  // Kp, Ki, Kd, maximum value of Isum, minimum value of p to calc Isum
  void set_pid_gain(float kp, float ki, float kd, float i_min = 0,
                    float i_max = FLT_MAX) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    i_min_ = i_min;
  }

  bool control() {
    // bool return_value = false;
    uint32_t dt = tutrcos::core::Kernel::get_ticks() - prev_ticks_;
    switch (mode_) {
    case Control::PI_D_RAD: {
      x_ = get_rad();
      p_ = ref_ - x_;
      i_ += p_ * (dt / tickrate_);
      i_ = (abs(p_) < i_min_) ? 0 : std::clamp<float>(i_, -i_max_, i_max_);
      d_ = (pre_x_ - x_) / (dt / tickrate_);
      input_ = (p_ * kp_ + i_ * ki_ + d_ * kd_);
      pre_x_ = x_;
      // set_input(input_ * dir_);
      set_input(input_);
      break;
    }
    case Control::RAW: {
      i_ = 0;
      // set_input(ref_ * dir_);
      set_input(ref_);
      break;
    }
    case Control::ZERO: {
      i_ = 0;
      set_input(0);
      break;
    }
    default: {
      i_ = 0;
      break;
    }
    }
    prev_ticks_ += dt;
    return true;
  }

protected:
private:
  const float tickrate_ = 1000;

  const int8_t i_dir_; // input_dir
  const int8_t o_dir_; // output_dir
  const float reduction_ratio_;
  const Turn turn_mode_;
  const Type rotater_type_;
  const int64_t cut_point_;
  Control mode_;
  EncoderBase *motor_;
  EncoderBase *enc_;

  int64_t count_;
  int64_t offset_rotation_;
  float input_;

  uint32_t prev_ticks_;
  uint32_t prev_ticks_ref_;

  float prev_p_;
  float p_, i_, d_;
  float kp_, ki_, kd_, i_max_, i_min_;
  float x_, pre_x_;

  float ref_;
  float ref_vel_limit_ = FLT_MAX;
  float ref_limit_min_ = -FLT_MAX;
  float ref_limit_max_ = FLT_MAX;
  float pre_ref_ = 0;
}; // namespace klibrary
} // namespace klibrary