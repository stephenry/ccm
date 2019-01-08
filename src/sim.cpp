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

void Sim::add_actor(CoherentActor * a) {
  actors_.insert(std::make_pair(a->id(), a));
}

void Sim::run() {
  FixedLatencyInterconnectModel interconnect_model{10};

  Frontier f;
  do {

    // Evaluate all actors in the platform
    //
    for (auto [t, actor] : actors_)
      actor->eval(f);

    // Forward all resultant transaction to target actors.
    //
    if (f.pending_transactions()) {
      interconnect_model.apply(f);
 
      TimeStamped<const Message *> head;
      while (f.pop(head)) {
        set_time(head.time());
          
        const Message * t = head.t();
        actors_[t->dst_id()]->apply(time(), t);
      }
    }
  } while (has_active_actors());
  
  // All actors are inactive
}

bool Sim::has_active_actors() const {
  for (auto [t, actor] : actors_) {
    if (actor->is_active())
      return true;
  }
  return false;
}

} // namespace ccm
