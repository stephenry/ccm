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

#include "scheduler.hpp"
#include "process.hpp"
#include "module.hpp"

namespace ccm::kernel {

  Frontier::Frontier() {
  }

  Scheduler::Scheduler() {
  }

  Scheduler::~Scheduler() {
    delete top_;
  }

  void Scheduler::run(RunOptions const & run_options) {
    CCM_ASSERT(top_ != nullptr);
    
    //
    set_state(SimState::Elaboration);
    if (top_) {
      top_->call_on_elaboration();
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
    
      for (Frontier::Task * p : frontier_.next()) {
        p->apply();
        p->release();
      }

      do_next_delta();
      while (!next_delta_.empty()) {
        ++delta_;
        do_next_delta();
      }
      frontier_.advance();
    }

    //
    set_state(SimState::Termination);
    if (top_)
      top_->call_on_termination();
  }

  void Scheduler::set_top (Module * ptr) { top_ = ptr; }

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
    //  add_task_wake_after(p, 0);
  }

  void Scheduler::add_frontier_task(Frontier::Task * t) {
    frontier_.add_work(t->time(), t);
  }
  void Scheduler::add_process_next_delta(Process * p) {
    next_delta_.push_back(p);
  }

  // void Scheduler::add_task_wake_on(Process * p, Event e) {
  //   //  e.add_to_wait_set(p);
  // }

  // void Scheduler::add_task_wake_after(Process * p, std::size_t time) {
  //   // if (time == 0) {
  //   //   next_delta_.push_back(p);
  //   // } else {
  //   //   FrontierTaskPtr task{new WakeProcessTask(p)};
  //   //   frontier_.add_work(time, std::move(task));
  //   // }
  // }

  // void Scheduler::add_task_notify_on(Event e, std::size_t time) {
  //   //  FrontierTaskPtr task{new NotifyEventTask(e)};
  //   //  frontier_.add_work(time, std::move(task));
  // }

  // void Scheduler::add_task_notify_after(Event e, std::size_t time) {
  //   //  FrontierTaskPtr task{new NotifyEventTask(e)};
  //   //  frontier_.add_work(now() + time, std::move(task));
  // }

  // Event Scheduler::create_event() {
  //   EventDescriptorPtr ed{new EventDescriptor{this}};
  //   Event e{ed.get()};
  //   events_.push_back(std::move(ed));
  //   return e;
  // }

  // Event Scheduler::create_event(EventOrList const & el) {
  //   EventDescriptorPtr ed{new EventOrDescriptor{this, el}};
  //   Event e{ed.get()};
  //   events_.push_back(std::move(ed));
  //   return e;
  // }

} // namespace ccm::kernel
