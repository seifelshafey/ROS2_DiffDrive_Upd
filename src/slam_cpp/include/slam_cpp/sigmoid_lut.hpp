#ifndef SLAM_CPP__SIGMOID_LUT_HPP_
#define SLAM_CPP__SIGMOID_LUT_HPP_

// sigmoid_lut.hpp — 256-entry sigmoid lookup table for int8 log-odds grids.
// Precomputes sigmoid(i / scale) for all int8 values [-128, 127].
// Runtime lookup is a single array access with zero math.
// Correctly unscales by SCALE before sigmoid (fixes Python scaling error).

#include <array>
#include <cmath>
#include <cstdint>

namespace slam {

// Built once at startup from the runtime scale parameter (default 20.0).
// 256 doubles = 2 KB, fits entirely in L1 cache.
class SigmoidLUT {
public:
  explicit SigmoidLUT(double scale = 20.0)
  {
    for (int i = 0; i < 256; ++i) {
      // Cast index to int8_t via uint8_t reinterpretation (maps 0..255 to -128..127).
      auto raw = static_cast<int8_t>(static_cast<uint8_t>(i));
      double log_odds = static_cast<double>(raw) / scale;
      table_[i] = 1.0 / (1.0 + std::exp(-log_odds));
    }
  }

  // Runtime lookup: uint8_t cast maps [-128,127] to [0,255] as index.
  double operator()(int8_t log_odds_scaled) const
  {
    return table_[static_cast<uint8_t>(log_odds_scaled)];
  }

private:
  std::array<double, 256> table_;
};

}  // namespace slam

#endif  // SLAM_CPP__SIGMOID_LUT_HPP_
