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

#ifndef __BUILDER_HPP__
#define __BUILDER_HPP__

#include "module.hpp"
#include <unordered_map>
#include <memory>

#define CCM_BUILDABLE_COMMON(__cls)                                     \
  friend class ::ccm::kernel::BuildableRegistry;                        \
  struct Factory : ::ccm::kernel::BuildableFactory {                    \
    const char * name() const override { return #__cls; }               \
    ::ccm::kernel::Buildable * construct(                               \
                    const ::ccm::kernel::Context & ctxt,                \
                    ::ccm::kernel::BuildableArguments & args) override {\
      using arg_type = typename __cls::Arguments;                       \
      return new __cls(ctxt, static_cast<arg_type &>(args));            \
    }                                                                   \
  };                                                                    \

namespace ccm::kernel {

  struct BuildableArguments {
    BuildableArguments(std::size_t id, const std::string & instance_name)
      : id_(id), instance_name_(instance_name)
    {}
    std::size_t id_;
    std::string instance_name_;
  };

  struct Buildable : public Module {
    Buildable(const Context & ctxt);
  };

  struct BuildableFactory {
    virtual const char * name() const = 0;
    virtual Buildable* construct(const Context & ctxt, BuildableArguments & opts) = 0;
  };

  class BuildableRegistry {
  public:
    template<typename T>
    void register_agent() {
      using factory_type = typename T::Factory;
      factory_type * factory = new factory_type{};
      buildables_[factory->name()] = factory;
    }
    void register_agent(std::string name, BuildableFactory * f);
    Buildable * construct(const Context & ctxt, std::string name, BuildableArguments & args);

  private:
    std::unordered_map<std::string, BuildableFactory *> buildables_;
  };

} // namespace ccm::kernel

#endif
