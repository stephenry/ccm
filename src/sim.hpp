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
    std::cout << " time = " << t
              << " message = " << to_string(*msg)
              << "\n";
    
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

  void add_actor(CoherentActor * a);

  void run();

 private:
  void set_time(std::size_t time) { time_ = time; }
  std::size_t time() const { return time_; }
  
  bool has_active_actors() const;

  std::size_t time_;
  std::map<std::size_t, CoherentActor *> actors_;
};

} // namespace ccm

#endif
