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
  NotifyEventTask(EventHandle e) : e_(e) {}
  ~NotifyEventTask() {}
  bool is_nop() const override { return false; }
  void apply(Scheduler * sch) override { e_.wake_waiting_processes(); }
 private:
  EventHandle e_;
};

struct WakeProcessTask : FrontierTask {
  WakeProcessTask(Process * p) : p_(p) {}
  bool is_nop() const override { return false; }
  void apply(Scheduler * sch) override { sch->add_task_wake_after(p_); }
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
  set_state(SimState::Elaboration);
  if (top_) {
    ElaborationState const state{this};
    top_->call_on_elaboration(state);
  }

  //
  set_state(SimState::Initialization);
  if (top_)
    top_->call_on_initialization();

  //
  set_state(SimState::Running);
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
      p->cb__on_termination();
    }
    reaped_processes_.clear();
  }

  //
  set_state(SimState::Termination);
  if (top_)
    top_->call_on_termination();
}

void Scheduler::do_next_delta() {
  std::swap(current_delta_, next_delta_);
  for (Process * p : current_delta_) {
    switch (state()) {
    case SimState::Initialization: p->call_on_initialization(); break;
    case SimState::Running: p->call_on_invoke(); break;
    }
  }
}

void Scheduler::add_process (Process * p) {
  add_task_wake_after(p, 0);
}

void Scheduler::add_task_next_delta(Process * p) {
  next_delta_.push_back(p);
}

void Scheduler::add_task_wake_on(Process * p, EventHandle e) {
  e.add_to_wait_set(p);
}

void Scheduler::add_task_wake_after(Process * p, std::size_t time) {
  if (time == 0) {
    next_delta_.push_back(p);
  } else {
    FrontierTaskPtr task{new WakeProcessTask(p)};
    frontier_.add_work(time, std::move(task));
  }
}

void Scheduler::add_task_notify_after(EventHandle e, std::size_t time) {
  FrontierTaskPtr task{new NotifyEventTask(e)};
  frontier_.add_work(time, std::move(task));
}

EventHandle Scheduler::create_event() {
  EventDescriptorPtr ed{new EventDescriptor{this}};
  EventHandle e{ed.get()};
  events_.push_back(std::move(ed));
  return e;
}

EventHandle Scheduler::create_event(EventOrList const & el) {
  EventDescriptorPtr ed{new EventOrDescriptor{this, el}};
  EventHandle e{ed.get()};
  events_.push_back(std::move(ed));
  return e;
}

} // namespace ccm
