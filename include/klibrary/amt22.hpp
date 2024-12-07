#pragma once

#include "main.h"

#include <array>
#include <cstdint>

#include <tutrcos/module/amt22.hpp>
#include <tutrcos/module/encoder_base.hpp>

namespace klibrary {

class AMT22 : public tutrcos::module::AMT22 {
public:
  AMT22(tutrcos::peripheral::SPI &spi, tutrcos::peripheral::GPIO &cs,
        tutrcos::module::AMT22::Resolution resolution,
        tutrcos::module::AMT22::Mode mode, float cut_point = 0.5)
      : tutrcos::module::AMT22{spi, cs, resolution, mode}, mode_{mode},
        ppr_{1 << tutrcos::utility::to_underlying(resolution)},
        cut_point_{static_cast<int64_t>(ppr_ * cut_point)} {}

  bool update() override {
    int64_t count;
    if (!tutrcos::module::AMT22::update()) {
      return false;
    }
    if (mode_ == Mode::SINGLE_TURN) {
      count = get_count();
      if (count > cut_point_) {
        count -= ppr_;
        set_count(count);
      }
    }
    return true;
  }

private:
  tutrcos::module::AMT22::Mode mode_;
  const int32_t ppr_;
  const int64_t cut_point_;
};

} // namespace klibrary
