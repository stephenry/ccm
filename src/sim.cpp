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
#include "coherence.hpp"
#include "interconnect.hpp"
#include "message.hpp"
#include "actor.hpp"

namespace ccm {

std::string to_string(const Time &t) {
  std::stringstream ss;
  ss << static_cast<unsigned>(t);
  return ss.str();
}

void Cursor::advance(std::size_t steps) { set_time(time() + (step() * steps)); }

Epoch::Epoch(Time start, Time duration, Time step)
    : start_(start), duration_(duration), step_(step), cursor_(start) {}

bool Epoch::in_interval(Time t) const { return (t < end()); }

Epoch Epoch::advance() const { return Epoch{end(), duration(), step()}; }

std::string to_string(const Epoch &epoch) {
  using namespace std;

  StructRenderer sr;
  sr.add("start", to_string(epoch.start()));
  sr.add("end", to_string(epoch.end()));
  sr.add("step", to_string(epoch.step()));
  return sr.str();
}

void Context::emit_message(TimeStamped<Message *> msg) { msgs_.push_back(msg); }

bool RunOptions::has_completed(Time current) const {
  bool ret{false};
  switch (terminate()) {
    case Terminate::OnExhaustion:
      ret = false;
      break;
    case Terminate::AfterTime:
      ret = (current >= final_);
      break;
    default:
      ret = true;
      break;
  }
  return ret;
}

Sim::Sim() : time_(0) {}
Sim::~Sim() {}

void Sim::add_actor(std::unique_ptr<CoherentActor> && actor) {
  actors_.insert(std::make_pair(actor->id(), std::move(actor)));
}

void Sim::add_interconnect(
    std::unique_ptr<InterconnectModel> && interconnect) {
  interconnect_ = std::move(interconnect);
}

void Sim::run(const RunOptions &run_options) {
  Epoch current_epoch{0, epoch_period_, epoch_step_};
  do {
    if (run_options.has_completed(current_epoch.start())) break;

    Context ctxt{current_epoch};
    for (auto it = actors_.begin(); it != actors_.end(); ++it)
      it->second->eval(ctxt);

    for (TimeStamped<Message *> ts : ctxt.msgs()) {
      interconnect_->apply(ts);

      const Message *msg = ts.t();
      actors_[msg->dst_id()]->apply(ts);
    }
    current_epoch = current_epoch.advance();
  } while (has_active_actors());
}

bool Sim::has_active_actors() const {
  for (auto it = actors_.begin(); it != actors_.end(); ++it)
    if (it->second->is_active()) return true;
  return false;
}

}  // namespace ccm
