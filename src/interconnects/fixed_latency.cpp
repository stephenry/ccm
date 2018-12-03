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

#include "fixed_latency.hpp"
#include "kernel/kernel.hpp"
#include <string>

namespace ccm {

  struct FixedLatency::PushProcess : kernel::Process {
    PushProcess(std::size_t id, FixedLatency * fl)
      : id_(id), fl_(fl)
    {}
  private:
    virtual void cb__on_invoke() override {
      kernel::Transaction * t;
      if (fl_->ins_[id_]->get(t)) {
        fl_->eqs_[t->portid_dst]->set(t, now() + fl_->arg_.latency);
      }
    }
    std::size_t id_;
    FixedLatency * fl_;
  };
  
  struct FixedLatency::PopProcess : kernel::Process {
    PopProcess(std::size_t id, FixedLatency * fl)
      : id_(id), fl_(fl)
    {}
  private:
    virtual void cb__on_invoke() override {
      kernel::Transaction * t;
      if (fl_->eqs_[id_]->get(t, now())) {
        fl_->outs_[id_]->push(t);
      }
    }
    std::size_t id_;
    FixedLatency * fl_;
  };
  

  FixedLatency::FixedLatency(const Arguments & arg)
    : arg_(arg) {

    for (std::size_t i = 0; i < arg_.in_ports; i++) {
      p_push_.push_back(create_process<PushProcess>(i, this));
      ins_.push_back(create_child<kernel::TMailBox>(std::string("in") + std::to_string(i)));
    }

    for (std::size_t i = 0; i < arg_.out_ports; i++) {
      event_queue_type * eq = create_child<kernel::EventQueue<kernel::Transaction *>>();
      PopProcess * p = create_process<PopProcess>(i, this);
      p->set_sensitive_on(eq->event());
      p_pop_.push_back(p);
      eqs_.push_back(eq);
    }
  }

  void FixedLatency::cb__on_initialization() {
    for (std::size_t i = 0; i < p_push_.size(); i++) {
      p_push_[i]->set_sensitive_on(ins_[i]->event());
    }
  }

} // namespace ccm

