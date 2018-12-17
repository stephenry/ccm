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

#include "process.hpp"
#include "scheduler.hpp"

namespace ccm::kernel {

Process::Process (const Context & context)
    : Object(context) {
  sensitive_.resize(1);
}

void Process::wait(Event e) {
  sensitive_.push_back(Sensitive{e});
}

void Process::wait_for(std::size_t t) {
  wait_until(context().now() + t);
}

void Process::wait_until(std::size_t t) {
  sensitive_.push_back(Sensitive{SensitiveOn::TimeAbsolute, t});
}

void Process::kill() {
  is_running_ = false;
}

void Process::call_on_elaboration() {
  cb__on_elaboration();
}

void Process::call_on_initialization() {
  cb__on_initialization();
  //  apply_sensitivity(sensitive_.back());
  //  sensitive_.resize(1);
}

void Process::call_on_invoke() {
  cb__on_invoke();
  if (!is_running_) {
    log_debug("Process is terminated.");
    return;
  }
  apply_sensitivity(sensitive_.back());
  sensitive_.resize(1);
}

void Process::apply_sensitivity(const Sensitive & s) {
  std::size_t t = s.t;
  if (s.on == SensitiveOn::Event) {
    Event e = s.e;
    e.add_to_wait_set(this);
  } else {
    struct WakeProcess : Task {
      WakeProcess(Scheduler * sch, Process * p, std::size_t t)
          : p_(p), sch_(sch), t_(t) {}
      void apply() override { sch_->add_process_next_delta(p_); }
      std::size_t time() const override { return t_; }
      std::string what() const override {
        std::stringstream ss;
        ss << "WakeProcess " << p_->name() << " at: " << time();
        return ss.str();
      }
     private:
      Scheduler * sch_;
      Process * p_;
      std::size_t t_;
    };
    if (s.on == SensitiveOn::TimeRelative)
      t += context().now();
    Scheduler * sch = context().sch();
    sch->add_frontier_task(std::make_unique<WakeProcess>(sch, this, t));
  }
}

//
void Process::call_on_termination() {
  cb__on_termination();
}

//
void Process::set_sensitive_on(Event e) {
  sensitive_[0] = Sensitive{e};
  is_running_ = true;
  apply_sensitivity(sensitive_[0]);
}

void Process::set_periodic(std::size_t t) {
  sensitive_[0] = Sensitive{SensitiveOn::TimeRelative, t};
  is_running_ = true;
  apply_sensitivity(sensitive_[0]);
}

} // namespace ccm::kernel
