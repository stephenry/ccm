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

#include "kernel.hpp"

#include <vector>
#include <map>

namespace ccm {

struct FrontierTask;
using FrontierTaskPtr = std::unique_ptr<FrontierTask>;



struct SchedulerImpl {
  //
  void add_process (Process * p);
  void remove_process (Process * p);
  //
  void run();
  bool run_delta();
  bool run_advance_time();

  //
  void add_to_runnable_set(Process * p) {
    next_delta_.push_back(p);
  }
  void add_to_frontier(FrontierTaskPtr p) {
    frontier_[p->time()].push_back(p);
  }

  SimState state() const { return state_; }
  void update_state(SimState ss) { state_ = ss; }

  std::size_t now() const { return now_; }

  //
};

FrontierTaskPtr FrontierTask::from_invoke_rsp(const InvokeRsp & rsp) {
  FrontierTaskPtr p;

  return p;
}

struct Event::Descriptor {
  void notify() {
    for (Process * p : pw_)
      sch_->add_to_runnable_set(p);
  }
  void add_to_wait_set(Process * p) {
    pw_.push_back(p);
  }
 private:
  std::vector<Process *> pw_;
  SchedulerImpl * sch_;
};

void Event::notify() { d_->notify(); }
void Event::add_to_wait_set(Process * p) { d_->add_to_wait_set(p); }

Scheduler::Scheduler() {
  impl_ = std::make_unique<SchedulerImpl>();
}

Scheduler::~Scheduler() {
}

SimState Scheduler::state() const { return impl_->state(); }
std::size_t Scheduler::now() const { return impl_->now(); }

Event Scheduler::create_event() {
  std::unique_ptr<Event::Descriptor> ed =
      std::make_unique<Event::Descriptor>();
  Event e{ed.get()};
  impl_->events_.push_back(std::move(ed));
  return e;
}

void Scheduler::add_process(Process * p) {
  impl_->add_process(p);
}
  
void Scheduler::remove_process(Process * p) {
}

void Scheduler::run() { impl_->run(); }

void SchedulerImpl::add_process(Process * p) {
  next_delta_.push_back(p);
}

void SchedulerImpl::run() {
  update_state(SimState::Elaboration);
    
  update_state(SimState::Initialization);

  now_ = 0;
  while (run_delta())
    delta_++;
    
  update_state(SimState::Running);

  while (run_advance_time())
    ;
    
  update_state(SimState::Termination);
}

bool SchedulerImpl::run_delta() {
  std::swap(current_delta_, next_delta_);
  next_delta_.clear();
  for (Process * p : current_delta_) {
    InvokeReq req(this);
    const InvokeRsp rsp = p->invoke(req);
    switch (rsp.type()) {
      case ResponseType::WakeOn: {
        Event e = rsp.event();
        e.add_to_wait_set(p);
      } break;
      case ResponseType::WakeAfter: {
        add_to_frontier(rsp.time(), FrontierTask::from_invoke_rsp(rsp));
      } break;
      case ResponseType::NotifyAfter: {
        add_to_frontier(rsp.time(), FrontierTask::from_invoke_rsp(rsp));
      } break;
      case ResponseType::Terminate: {
      } break;
    }
  }

  return (next_delta_.size() != 0);
}

bool SchedulerImpl::run_advance_time() {
  if (frontier_.size() == 0)
    return false;

  auto it = frontier_.begin();
  now_ = it->first;
  //  next_delta_ = std::move(it->second);
  frontier_.erase(it);
  while (run_delta())
    delta_++;

  return (frontier_.size() == 0);
}

SimState InvokeReq::state() const { return sch_->state(); }
std::size_t InvokeReq::now() const { return sch_->now(); }
  
} // namespace ccm
