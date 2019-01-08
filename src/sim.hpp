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
  
using Time = std::size_t;

struct Epoch {
  Epoch(Time start, Time end, Time step);

  bool in_interval(Time t) const;
  Epoch advance() const;
  void step();

  Time now() const { return now_; }
  Time start() const { return start_; }
  Time end() const { return end_; }
  Time step() const { return step_; }

private:
  Time start_, end_, step_, now_;
};
  
struct Context {

  Context(const Epoch & epoch)
    : epoch_(epoch)
  {}

  Epoch epoch() const { return epoch_; }

  void emit_message(Time t, const Message * m) {
    msgs_.push_back(TimeStamped{t, m});
  }

  //private:
  std::vector<TimeStamped<const Message *> > msgs_;
  Epoch epoch_;
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
