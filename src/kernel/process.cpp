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

//
std::size_t Process::now() const { return sch_->now(); }
std::size_t Process::delta() const { return sch_->delta(); }

//
void Process::wait(EventHandle e) {
  s_dynamic_ = Sensitive{SensitiveTo::Dynamic, e};
}

//
void Process::wait_for(std::size_t t) {
  wait_until(now() + t);
}

//
void Process::wait_until(std::size_t t) {
  s_dynamic_ = Sensitive{SensitiveTo::Dynamic, t};
}

//
void Process::call_on_elaboration(ElaborationState const & state) {
  set_scheduler(state.sch);
  cb__on_elaboration();
}

//
void Process::call_on_initialization() {
  cb__on_initialization();
}

//
void Process::call_on_invoke() {
  s_dynamic_ = Sensitive{};
  cb__on_invoke();
  if (s_dynamic_.is_valid) {
    apply_sensitivity(s_dynamic_);
  } else {
    apply_sensitivity(s_static_);
  }
}

//
void Process::apply_sensitivity(Sensitive s) {
  switch (s.on) {
  case SensitiveOn::Event: {
    sch_->add_task_wake_on(this, s.e);
  } break;
  case SensitiveOn::Time: {
    if (now() == s.t) {
      sch_->add_task_next_delta(this);
    } else {
      sch_->add_task_wake_after(this, s.t);
    }
  }
  }
}

//
void Process::call_on_termination() {
  cb__on_termination();
}

//
void Process::set_sensitive_on(EventHandle e) {
  s_static_ = Sensitive{SensitiveTo::Static, e};
  sch_->add_task_wake_on(this, e);
}

} // namespace ccm::kernel
