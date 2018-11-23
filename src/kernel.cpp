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


  struct SchedulerImpl {
    //
    void add_process (Process * p);
    void remove_process (Process * p);
    //
    void run();
    bool run_delta();
    bool run_advance_time();

    SimState state() const { return state_; }
    std::size_t now() const { return now_; }

    //
    std::vector<std::unique_ptr<EventDescriptor>> events_;
    std::vector<Process *> current_delta_, next_delta_;
    std::map<std::size_t, std::vector<Process *>> frontier_;
    SimState state_{SimState::Initialization};
    std::size_t delta_{0};
    std::size_t now_{0};
  };

  struct EventDescriptor {
    std::vector<Process *> processes_awaiting_;
  };
  
  Scheduler::Scheduler() {
    impl_ = std::make_unique<SchedulerImpl>();
  }

  Scheduler::~Scheduler() {
  }

  SimState Scheduler::state() const { return impl_->state(); }
  std::size_t Scheduler::now() const { return impl_->now(); }

  Event Scheduler::create_event() {
    std::unique_ptr<EventDescriptor> ed =
      std::make_unique<EventDescriptor>();
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
    state_ = SimState::Elaboration;
    
    state_ = SimState::Initialization;

    now_ = 0;
    while (run_delta())
      delta_++;
    
    state_ = SimState::Running;

    while (run_advance_time())
      ;
    
    state_ = SimState::Termination;
  }

  bool SchedulerImpl::run_delta() {
    std::swap(current_delta_, next_delta_);
    for (Process * p : current_delta_) {
      InvokeReq req(this);
      const InvokeRsp rsp = p->invoke(req);
      switch (rsp.wake_) {
      case Wake::WEvent: {
        rsp.e_.ed_->processes_awaiting_.push_back(p);
      } break;
      case Wake::WTime: {
        if (p == 0)
          next_delta_.push_back(p);
        else
          frontier_[rsp.t_].push_back(p);          
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
    next_delta_ = std::move(it->second);
    frontier_.erase(it);
    while (run_delta())
      delta_++;

    return (frontier_.size() == 0);
  }

  SimState InvokeReq::state() const { return sch_->state(); }
  std::size_t InvokeReq::now() const { return sch_->now(); }
  
} // namespace ccm
