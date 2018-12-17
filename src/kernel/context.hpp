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

#ifndef __CONTEXT_HPP__
#define __CONTEXT_HPP__

#include "event.hpp"
#include <string>

namespace ccm::kernel {

class Scheduler;
class Module;

class Context {
  friend class Scheduler;
  friend class Module;
  friend class TopModule;
    
  Context(Scheduler * sch, Module * parent, const std::string & instance_name)
      : sch_(sch), parent_(parent), instance_name_(instance_name) {}

  Context create_child(Module * self, const std::string & instance_name) {
    return Context{sch_, self, instance_name};
  }
    
 public:
  std::size_t now() const;
  std::size_t delta() const;
  std::string instance_name() const;
  EventBuilder event_builder() const;
  Scheduler * sch() const;

 private:
  Scheduler * sch_;
  std::string instance_name_;
  Module * parent_{nullptr};
};

} // namespace ccm::kernel

#endif
