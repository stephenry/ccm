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

#ifndef __BASIC_HPP__
#define __BASIC_HPP__

#include "kernel/kernel.hpp"

namespace ccm::agents {

  class BasicSourceAgent : public ::ccm::kernel::Buildable {
    struct EmitProcess;
  public:
    BasicSourceAgent(const ::ccm::kernel::Context & ctxt, std::size_t period);

    ::ccm::kernel::TMailBoxIf * out_;
  protected:
    virtual ::ccm::kernel::Transaction * source_transaction() = 0;
  private:
    EmitProcess * p_;
    std::size_t period_;
  };

  class BasicSinkAgent : public ::ccm::kernel::Buildable {
    struct ConsumeProcess;
  public:
    BasicSinkAgent(const ::ccm::kernel::Context & ctxt);
    
    ::ccm::kernel::TMailBox * in_;
  protected:
    virtual void sink_transaction (::ccm::kernel::Transaction * t) = 0;
  private:

    ConsumeProcess * p_;
  };

} // namespace ccm::agents

#endif
