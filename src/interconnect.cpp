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

#include "interconnect.hpp"
#include "message.hpp"
#include "sim.hpp"

namespace ccm {
#ifdef ENABLE_JSON

std::unique_ptr<InterconnectModel>
InterconnectModel::from_json(nlohmann::json & j) {
  if (j["type"] == "fixedlatency")
    return FixedLatencyInterconnectModel::from_json(j["options"]);
  return nullptr;
}
#endif
#ifdef ENABLE_JSON

std::unique_ptr<InterconnectModel>
FixedLatencyInterconnectModel::from_json(nlohmann::json & j) {
  const std::size_t latency = j["latency"];
  return std::unique_ptr<InterconnectModel>(
      new FixedLatencyInterconnectModel{latency});
}
#endif

InterconnectModel::~InterconnectModel() {}

void InterconnectModel::apply(TimeStamped<Message *> &ts) {
  update_time(ts);
}

void InterconnectModel::update_time(TimeStamped<Message *> &ts) {
  const Message *m = ts.t();
  ts.set_time(ts.time() + cost(m->src_id(), m->dst_id()));
}

FixedLatencyInterconnectModel::~FixedLatencyInterconnectModel() {}

}  // namespace ccm
