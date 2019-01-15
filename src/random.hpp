//========================================================================== //
// Copyright (c) 2018, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

#ifndef __SRCS_RANDOM_HPP__
#define __SRCS_RANDOM_HPP__

#include <limits>
#include <random>

namespace ccm {

struct Random {
  template <typename T>
  struct UniformRandomInterval {
    UniformRandomInterval(T hi = std::numeric_limits<T>::max(),
                          T lo = std::numeric_limits<T>::min())
        : hi_(hi), lo_(lo) {
      mt_ = std::mt19937{Random::rd_()};
      dst_ = std::uniform_int_distribution<T>(lo_, hi_);
    }

    T operator()() { return dst_(mt_); }

   private:
    std::mt19937 mt_;
    std::uniform_int_distribution<T> dst_;
    const T lo_, hi_;
  };

  static void set_seed(std::size_t seed) { Random::seed_ = seed; }

 private:
  static std::random_device rd_;
  static std::size_t seed_;
};

}  // namespace ccm

#endif