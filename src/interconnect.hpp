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

#ifndef __INTERCONNECT_HPP__
#define __INTERCONNECT_HPP__

#include "transaction.hpp"
#include "agents.hpp"
#include "kernel/kernel.hpp"

#include <unordered_map>
#include <memory>

#define CCM_REGISTER_INTERCONNECT(__name, __if)                         \
  ::ccm::InterconnectRegisterer __reg_ ## __af(__name, std::make_unique<__if>());

namespace ccm {

  //
  class Interconnect;
  using InterconnectPtr = std::unique_ptr<Interconnect>;

  //
  class InterconnectFactory;
  using InterconnectFactoryPtr = std::unique_ptr<InterconnectFactory>;

  //
  class InterconnectOptions {
  };

  //
  class Interconnect : public ccm::kernel::Module {
  public:
    virtual void push (Transaction * t) = 0;
    virtual Transaction * pop () = 0;

    // TODO: notify event on agent and pass back opaque token.
    virtual void register_agent (Agent * a) = 0;
  };

  //
  class InterconnectFactory {
  public:
    virtual InterconnectPtr construct (InterconnectOptions const & opts) = 0;
  };

  class InterconnectRegistry {
  public:
    static bool has_factory (char const * name);
    static InterconnectFactory * factory (char const * name);
    static void register_interconnect (char const * name, InterconnectFactoryPtr && f);

  private:
    static std::unordered_map<char const *, InterconnectFactoryPtr> interconnect_;
  };

  //
  struct InterconnectRegisterer {
    InterconnectRegisterer (const char * name, InterconnectFactoryPtr && f);
  };

} // namespace ccm

#endif
