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

#ifndef __SRC_SIM_HPP__
#define __SRC_SIM_HPP__

#include "utility.hpp"
#include "message.hpp"
#include "transaction.hpp"
#include "actors.hpp"
#include "interconnect.hpp"
#include <vector>
#include <map>

namespace ccm {

class Frontier {
 public:

  std::vector<TimeStamped<const Message *>> & ts() { return msgs_.ts(); }

  void add_to_frontier(std::size_t t, const Message * msg) {
    msgs_.push(make_time_stamped(t, msg));
  }

  bool pending_transactions() const {
    return !msgs_.empty();
  }

  bool pop(TimeStamped<const Message *> & msg) {
    return msgs_.pop(msg);
  }

  void clear() { msgs_.clear(); }

  void heapify() {
    msgs_.heapify();
  }

 private:
  Heap<TimeStamped<const Message *> > msgs_;
};

struct Sim {

  Sim() : time_(0) {}

  void add_actor(CoherentActor * a) {
    actors_.insert(std::make_pair(a->id(), a));
  }
  
  void run() {
    FixedLatencyInterconnectModel interconnect_model{10};

    Frontier f;
    while (has_active_actors()) {

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
          t->release();
        }
        f.clear();
      }
    }
    // All actors are inactive
  }

 private:
  void set_time(std::size_t time) { time_ = time; }
  std::size_t time() const { return time_; }
  
  bool has_active_actors() const {
    for (auto [t, actor] : actors_) {
      if (actor->is_active())
        return true;
    }
    return false;
  }

  std::size_t time_;
  std::map<std::size_t, CoherentActor *> actors_;
};

} // namespace ccm

#endif
