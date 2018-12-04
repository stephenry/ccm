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

  struct WakeProcessAtTimeTask : Frontier::Task {
    void apply () override { sch_->add_process_next_delta(p_); }
    std::size_t time() const { return time_; }
    void reset() override { sch_ = nullptr; p_ = nullptr; }

    std::size_t time_;
    Scheduler * sch_;
    Process * p_;
  };

  Process::Process (const Context & context)
    : ctxt_(context) {
    sensitive_.resize(1);
  }

  //
  void Process::wait(Event e) {
    sensitive_.push_back(Sensitive{SensitiveTo::Dynamic, e});
  }

  //
  void Process::wait_for(std::size_t t) { wait_until(ctxt_.now() + t); }
  void Process::wait_until(std::size_t t) {
    sensitive_.push_back(Sensitive{SensitiveTo::Dynamic, t});
  }

  //
  void Process::call_on_elaboration() {
    cb__on_elaboration();
  }

  //
  void Process::call_on_initialization() {
    cb__on_initialization();
    update_sensitivity();
  }

  //
  void Process::call_on_invoke() {
    cb__on_invoke();
    update_sensitivity();
  }

  void Process::update_sensitivity() {
    static Pool<WakeProcessAtTimeTask> pool_;
    
    Sensitive & top = sensitive_.back();

    if (!top.is_valid)
      return ;
    
    if (top.on == SensitiveOn::Event) {
      top.e.add_to_wait_set(this);
    } else {
      Scheduler * sch = ctxt_.sch();
      
      WakeProcessAtTimeTask * p = pool_.alloc();
      p->time_ = top.t;
      p->sch_ = sch;
      p->p_ = this;
      sch->add_frontier_task(p);
    }

    if (top.to == SensitiveTo::Dynamic)
      sensitive_.pop_back();
  }

  //
  void Process::call_on_termination() {
    cb__on_termination();
  }

  //
  void Process::set_sensitive_on(Event e) {
    sensitive_[0] = Sensitive{SensitiveTo::Static, e};
  }

} // namespace ccm::kernel
