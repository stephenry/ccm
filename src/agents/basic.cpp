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

#include "basic.hpp"

namespace ccm::agents {

  struct BasicSourceAgent::EmitProcess : krn::Process {
    EmitProcess(BasicSourceAgent * agnt, std::size_t period)
      : agnt_(agnt), period_(period)
    {}
      
    void cb__on_initialization() override {
      wait_until(now() + period_);
    }
    void cb__on_invoke() override  {
      krn::Transaction * t = agnt_->source_transaction();
      if (t != nullptr) {
        agnt_->out_->push(t);
        wait_until(now() + period_);
      }
    }
  private:
    std::size_t period_;
    BasicSourceAgent * agnt_;
  };
  
  BasicSourceAgent::BasicSourceAgent(std::size_t period)
    : period_(period) {
    p_ = create_process<EmitProcess>(this, period);
  }

  struct BasicSinkAgent::ConsumeProcess : krn::Process {
    ConsumeProcess(BasicSinkAgent * agnt)
      : agnt_(agnt)
    {}
  private:
    virtual void cb__on_invoke() {
      krn::Transaction * t;
      if (agnt_->in_->get(t))
        agnt_->sink_transaction(t);
    }
    BasicSinkAgent * agnt_;
  };

  BasicSinkAgent::BasicSinkAgent() {
    in_ = create_child<krn::TMailBox>("in");
    p_ = create_process<ConsumeProcess>(this);
  }

  void BasicSinkAgent::cb__on_initialization() {
    p_->set_sensitive_on(in_->event());
  };

} // namespace ccm::agents
