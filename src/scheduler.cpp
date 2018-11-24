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

#include "src/scheduler.hpp"
#include "src/process.hpp"
#include "src/module.hpp"

namespace ccm {

struct NotifyEventTask : FrontierTask {
  NotifyEventTask(Event e) : e_(e) {}
  ~NotifyEventTask() {}
  bool is_nop() const override { return false; }
  void apply(Scheduler * sch) override { e_.notify(); }
 private:
  Event e_;
};

struct WakeProcessTask : FrontierTask {
  WakeProcessTask(Process * p) : p_(p) {}
  bool is_nop() const override { return false; }
  void apply(Scheduler * sch) override { sch->add_to_runnable_set(p_); }
 private:
  Process * p_;
};

Frontier::Frontier() {
}

Scheduler::Scheduler() {
}

Scheduler::~Scheduler() {
}

void Scheduler::run(RunOptions const & run_options) {
  //
  set_state(Elaboration);
  top_->elaboration();

  //
  set_state(Initialization);
  do_next_delta();

  //
  set_state(Running);

  while (frontier_.work_remains()) {
    const std::size_t next_time = frontier_.next_time();
    if (!run_options.can_run_at_time(next_time))
      break;

    now_ = next_time;
    delta_ = 0;

    current_delta_.clear();
    next_delta_.clear();
    
    for (FrontierTaskPtr & ft : frontier_.next())
      ft->apply(this);

    do_next_delta();
    while (!next_delta_.empty()) {
      ++delta_;
      do_next_delta();
    }
    frontier_.advance();

    // Cleanup reaped process
    for (Process * p : reaped_processes_) {
      InvokeReq req{this};
      req.set_reaped();
      p->invoke_termination(req);
    }
    reaped_processes_.clear();
  }

  //
  set_state(Termination);
  top_->termination();
}

void Scheduler::do_next_delta() {
  std::swap(current_delta_, next_delta_);
  for (Process * p : current_delta_) {
    InvokeReq req{this};
    const InvokeRsp rsp{
      (state() == SimState::Running) ? p->invoke_running(req) :
          p->invoke_initialization(req)};
    switch (rsp.type()) {
      case ResponseType::WakeOn: {
        Event e;
        e.add_to_wait_set(p);
      } break;
      case ResponseType::WakeAfter: {
        FrontierTaskPtr task{new WakeProcessTask(p)};
        frontier_.add_work(rsp.time(), std::move(task));
      } break;
      case ResponseType::NotifyAfter: {
        FrontierTaskPtr task{new NotifyEventTask(rsp.event())};
        frontier_.add_work(rsp.time(), std::move(task));
      } break;
      case ResponseType::Terminate: {
        reaped_processes_.push_back(p);
      } break;
    }
  }
}

void Scheduler::add_event(Event & e) {
  EventDescriptorPtr ed{new EventDescriptor{this}};
  e = Event{ed.get()};
  events_.push_back(std::move(ed));
}

void Scheduler::add_process (Process * p) {
  next_delta_.push_back(p);
}

void Scheduler::add_to_runnable_set(Process * p) {
  next_delta_.push_back(p);
}

} // namespace ccm
