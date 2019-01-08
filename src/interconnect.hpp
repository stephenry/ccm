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

#ifndef __SRC_INTERCONNECT_HPP__
#define __SRC_INTERCONNECT_HPP__

#include "utility.hpp"

namespace ccm {

class Frontier;
class Message;

struct InterconnectModel {
  InterconnectModel() {}
  virtual ~InterconnectModel() {}
  
  void apply(TimeStamped<const Message *> & ts);
 private:
  virtual std::size_t cost(std::size_t src_id, std::size_t dst_id) = 0;
  void update_time(TimeStamped<const Message *> & ts);
};

struct FixedLatencyInterconnectModel : InterconnectModel {
  FixedLatencyInterconnectModel(std::size_t latency)
      : latency_(latency)
  {}
 private:
  std::size_t cost(std::size_t src_id, std::size_t dst_id) override {
    return latency_;
  }
  std::size_t latency_;
};

} // namespace ccm

#endif
