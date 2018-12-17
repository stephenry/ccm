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


//
void Frontier::add_work(std::size_t t, std::unique_ptr<Task> && p) {
  f_[t].push_back(std::move(p));
}

bool Frontier::work_remains() const {
  return !f_.empty();
}

std::size_t Frontier::next_time() const {
  return f_.begin()->first;
}

std::vector<std::unique_ptr<Task> > & Frontier::next() {
  return f_.begin()->second;
}

void Frontier::advance() {
  f_.erase(f_.begin());
}

Frontier::Frontier() {
}

Scheduler::Scheduler() {
}

Scheduler::~Scheduler() {
}

void Scheduler::run(RunOptions const & run_options) {
  CCM_ASSERT(top_ != nullptr);
    
  //
  set_state(SimState::Elaboration);
  top_->call_on_elaboration();

  //
  set_state(SimState::Initialization);
  top_->call_on_initialization();

  //
  set_state(SimState::Running);
  while (frontier_.work_remains()) {
    const std::size_t next_time = frontier_.next_time();
    if (!run_options.can_run_at_time(next_time))
      break;

    now_ = next_time;

    next_delta_.clear();

    for (std::unique_ptr<Task> & p : frontier_.next())
      p->apply();

    delta_ = 0;
    do {
      current_delta_.clear();
      std::swap(current_delta_, next_delta_);

      for (Process * p : current_delta_)
        p->call_on_invoke();

      if (++delta_ == Scheduler::DELTA_MAX)
        break;

    } while (next_delta_.size() != 0);

    frontier_.advance();
  }

  //
  set_state(SimState::Termination);
  top_->call_on_termination();
}

void Scheduler::set_top(Module * ptr) {
  top_ = ptr;
}

void Scheduler::add_frontier_task(std::unique_ptr<Task> && task) {
  debug("Add task to frontier: ", task->what());
  frontier_.add_work(task->time(), std::move(task));
}
void Scheduler::add_process_next_delta(Process * p) {
  next_delta_.push_back(p);
}

std::string Scheduler::prefix() const {
  std::stringstream ss;
  ss << "[Scheduler: " << now_ << ":" << delta_ << "] ";
  return ss.str();
}

} // namespace ccm::kernel
