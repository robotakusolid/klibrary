#pragma once

#include "main.h"

#include "utility.hpp"
#include <cmath>
#include <tutrcos.hpp>
#include <tutrcos/module/encoder_base.hpp>

using namespace tutrcos;
using namespace tutrcos::module;

namespace klibrary {
class MotorBase : public EncoderBase {
#define FLT_MAX std::numeric_limits<float>::max()
public:
  enum class Dir : int8_t {
    FORWARD = 1,
    REVERSE = -1,
  };

  enum class ControlType { PI_D_RAD, PI_D_RADPS, RAW, ZERO };

  MotorBase(Dir dir, float reduction_ratio, int64_t cpr,
            EncoderBase *enc = nullptr)
      : EncoderBase(cpr), dir_{utility::to_underlying(dir)},
        reduction_ratio_{reduction_ratio} {
    enc_ = enc;
  }

  virtual bool update() { return control(); };
  virtual void stop() { set_input(0); }
  virtual void set_input(float value) { input_ = value; };

  float get_rotation() override {
    return static_cast<float>(get_count()) / get_cpr() * dir_;
  }

  void set_mode(ControlType mode) { mode_ = mode; }

  void set_count_w_dir(int64_t count) { set_count(count * dir_); }

  void set_offset_degree(float value) {
    offset_ = value / 360. / reduction_ratio_ * get_cpr() * dir_;
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
    case ControlType::PI_D_RAD: {
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
    case ControlType::RAW: {
      i_ = 0;
      // set_input(ref_ * dir_);
      set_input(ref_);
      break;
    }
    case ControlType::ZERO: {
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
  const int8_t dir_;
  int64_t offset_;
  EncoderBase *enc_;

private:
  const float tickrate_ = 1000;
  const float reduction_ratio_;

  ControlType mode_;
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
};
} // namespace klibrary