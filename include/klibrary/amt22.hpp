#pragma once

#include "main.h"

#include <array>
#include <cstdint>

#include <tutrcos/module/amt22.hpp>
#include <tutrcos/module/encoder_base.hpp>

namespace klibrary {

class AMT22 : public tutrcos::module::AMT22 {
public:
  enum class Dir : int8_t {
    FORWARD = 1,
    REVERSE = -1,
  };

  AMT22(tutrcos::peripheral::SPI &spi, tutrcos::peripheral::GPIO &cs,
        tutrcos::module::AMT22::Resolution resolution, Dir dir,
        tutrcos::module::AMT22::Mode mode = Mode::MULTI_TURN,
        float cut_point = 0.5)
      : tutrcos::module::AMT22{spi, cs, resolution, mode},
        dir_{tutrcos::utility::to_underlying(dir)}, mode_{mode},
        cpr_{1 << tutrcos::utility::to_underlying(resolution)},
        cut_point_{static_cast<int64_t>(cpr_ * cut_point)} {}

  bool update() {
    int64_t count;
    if (!tutrcos::module::AMT22::update()) {
      return false;
    }
    if (mode_ == Mode::SINGLE_TURN) {
      count = get_count();
      if (count > cut_point_) {
        count -= cpr_;
        set_count(count);
      }
    }
    return true;
  }

  float get_rotation() { return static_cast<float>(get_count()) / cpr_ * dir_; }

private:
  const int8_t dir_;
  tutrcos::module::AMT22::Mode mode_;
  const int64_t cpr_;
  const int64_t cut_point_;
};

} // namespace klibrary
