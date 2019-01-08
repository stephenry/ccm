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

#include "sim.hpp"

namespace ccm {

Epoch::Epoch(Time start, Time end, Time step)
  : start_(start), end_(end), step_(step), now_(start)
{}

bool Epoch::in_interval(Time t) const {
  return (start_ <= t) && (t < end_);
}

Epoch Epoch::advance() const {
  return Epoch{end_, end_ + step_, step_};
}

void Epoch::step() {
  now_ += step_;
}

void Sim::add_actor(CoherentActor * a) {
  actors_.insert(std::make_pair(a->id(), a));
}

void Sim::run() {
  FixedLatencyInterconnectModel interconnect_model{10};

  Epoch current_epoch{0, 100, 10};
  do {
    Context ctxt{current_epoch};
    for (auto [t, actor] : actors_)
      actor->eval(ctxt);

    for (TimeStamped<const Message *> ts : ctxt.msgs_) {
      interconnect_model.apply(ts);
        
      const Message * msg = ts.t();
      actors_[msg->dst_id()]->apply(ts);
    }
    current_epoch = current_epoch.advance();
  } while (has_active_actors());
}

bool Sim::has_active_actors() const {
  for (auto [t, actor] : actors_) {
    if (actor->is_active())
      return true;
  }
  return false;
}

} // namespace ccm
