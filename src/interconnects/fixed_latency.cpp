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

namespace ccm {

  class FixedLatency : public Interconnect {
    CCM_REGISTER_INTERCONNECT(FixedLatency);
  public:
    using arg_type = FixedLatencyArguments;
    
    FixedLatency(const arg_type & arg);
    
    void push (Transaction * t) override;
    void register_agent (std::size_t id, Agent * a) override;
  private:

    std::vector<kernel::EventQueue<Transaction *> *> eqs_;
    arg_type arg_;
  };

  FixedLatency::FixedLatency(const FixedLatencyArguments & arg)
    : arg_(arg) {
  }
    
  void FixedLatency::push (Transaction * t) {
    eqs_[t->portid_dst]->set(t, now() + arg_.latency);
  }
  
  void FixedLatency::register_agent (std::size_t id, Agent * a) {
    if (eqs_.size() < id)
      eqs_.resize(id);

    eqs_[id] = create_child<kernel::EventQueue<Transaction *>>();
  }

} // namespace ccm

