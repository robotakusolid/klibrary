#pragma once

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <tutrcos/core.hpp>
#include <type_traits>
#include <vector>

constexpr float deg2rad(float deg) { return deg / 180. * M_PI; }
constexpr float rad2Deg(float rad) { return rad * M_PI / 180.; }

namespace klibaray {
void print_vec(std::vector<uint8_t> data) {
  for (uint8_t d : data) {
    printf("%d, ", d);
  }
}

void print_vec(std::vector<float> data) {
  for (float d : data) {
    printf("%f, ", d);
  }
}

template <size_t N> void print_vec(std::array<float, N> data) {
  for (int i = 0; i < data.size(); i++) {
    printf("%f, ", data[i]);
  }
}

inline void delay(std::chrono::milliseconds ms) {
  tutrcos::core::Thread::delay(ms.count());
}

uint8_t calc_checksum(std::vector<uint8_t> data) {
  uint8_t checksum = 0;
  for (uint8_t i : data)
    checksum += i;
  return checksum;
}
} // namespace klibaray