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

namespace krn = ::ccm::kernel;

namespace ccm::interconnects {

  FixedLatencyTransaction * flt(krn::Transaction * t) {
    return static_cast<FixedLatencyTransaction *>(t);
  }

  struct FixedLatency::PushProcess : krn::Process {
    PushProcess(const krn::Context & ctxt,
                std::size_t id, FixedLatency * fl)
      : Process(ctxt), id_(id), fl_(fl)
    {}
    virtual ~PushProcess() {}
  private:
    virtual void cb__on_invoke() override {
      krn::Transaction * t;
      if (fl_->ins_[id_]->get(t)) {
        fl_->eqs_[flt(t)->portid_dst]->set(t, context().now() + fl_->args_.latency);
      }
    }
    std::size_t id_;
    FixedLatency * fl_;
  };
  
  struct FixedLatency::PopProcess : krn::Process {
    PopProcess(const krn::Context & ctxt,
               std::size_t id, FixedLatency * fl)
      : Process(ctxt), id_(id), fl_(fl)
    {}
    virtual ~PopProcess() {}
  private:
    virtual void cb__on_invoke() override {
      krn::Transaction * t;
      if (fl_->eqs_[id_]->get(t, context().now())) {
        fl_->outs_[id_]->push(t);
      }
    }
    std::size_t id_;
    FixedLatency * fl_;
  };
  
  FixedLatency::FixedLatency(const krn::Context & ctxt, const Arguments & args)
    : Buildable(ctxt), args_(args) {

    const std::string sin{"in"};
    for (std::size_t i = 0; i < args_.in_ports; i++) {
      ins_.push_back(create_child<krn::TMailBox>(sin + std::to_string(i)));
      p_push_.push_back(create_process<PushProcess>("PPush", i, this));
      p_push_.back()->set_sensitive_on(ins_.back()->event());
    }

    const std::string seq{"eq"};
    for (std::size_t i = 0; i < args_.out_ports; i++) {
      eqs_.push_back(create_child<krn::TEventQueue>(seq + std::to_string(i)));
      p_pop_.push_back(create_process<PopProcess>("PPop", i, this));
      p_pop_.back()->set_sensitive_on(eqs_.back()->event());
    }
    outs_.resize(args_.out_ports);
  }

  FixedLatency::~FixedLatency() {}
  
} // namespace ccm::interconnects
