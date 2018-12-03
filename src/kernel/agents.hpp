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

#ifndef __AGENTS_HPP__
#define __AGENTS_HPP__

#include "module.hpp"
#include "port.hpp"
#include <unordered_map>
#include <memory>

#define CCM_AGENT_COMMON(__cls)                                         \
  struct Factory : ::ccm::kernel::AgentFactory {                        \
    const char * name() const { return #__cls; }                        \
    ::ccm::kernel::AgentPtr construct(                                  \
                       ::ccm::kernel::AgentArguments & args) override { \
      using arg_type = typename __cls::Arguments;                       \
      return std::make_unique<__cls>(static_cast<arg_type &>(args));    \
    }                                                                   \
  };                                                                    \

namespace ccm::kernel {

  struct AgentArguments {
    std::size_t id;
    std::string instance_name;
  };

  struct Agent : public Module {
  };
  using AgentPtr = std::unique_ptr<Agent>;

  struct AgentFactory {
    virtual const char * name() const = 0;
    virtual AgentPtr construct(AgentArguments & opts) = 0;
  };

  class AgentRegistry {
  public:
    template<typename T>
    void register_agent() {
      using factory_type = typename T::Factory;
      factory_type * factory = new factory_type{};
      agents_[factory->name()] = factory;
    }
    void register_agent(std::string name, AgentFactory * f);
    Agent * construct(Module * m,
                      std::string name,
                      AgentArguments & args);

  private:
    std::unordered_map<std::string, AgentFactory *> agents_;
  };

} // namespace ccm::kernel

#endif
