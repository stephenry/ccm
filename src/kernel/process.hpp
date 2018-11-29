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

#ifndef __PROCESS_HPP__
#define __PROCESS_HPP__

#include "fwd.hpp"
#include "event.hpp"

namespace ccm::kernel {

enum class SensitiveTo { Static, Dynamic };
enum class SensitiveOn { Event, Time };

struct Sensitive {
  Sensitive()
    : is_valid(false)
  {}
  Sensitive(SensitiveTo to, EventHandle e)
    : is_valid(true), to(to), on(SensitiveOn::Event), e(e)
  {}
  Sensitive(SensitiveTo to, std::size_t t)
    : is_valid(true), to(to), on(SensitiveOn::Time), t(t)
  {}
  bool is_valid;
  SensitiveTo to;
  SensitiveOn on;
  union {
    EventHandle e;
    std::size_t t;
  };
};

class Process {
  friend class Scheduler;
  friend class Module;
  
 public:

  Process () : Process("<ANONYMOUS>") {}
  Process (std::string name) : name_(name) {}
  
  //
  virtual ~Process() {}

  //
  void set_sensitive_on(EventHandle e);

protected:
  
  //
  std::size_t now() const;
  std::size_t delta() const;

  //
  virtual void cb__on_elaboration() {}
  virtual void cb__on_initialization() {}
  virtual void cb__on_invoke() {}
  virtual void cb__on_termination() {}

  //
  void wait(EventHandle e);
  void wait_for(std::size_t t = 0);
  void wait_until(std::size_t t);

private:
  //
  void call_on_elaboration(ElaborationState const & state);
  void call_on_initialization();
  void call_on_invoke();
  void call_on_termination();
  //
  void set_scheduler(Scheduler * sch) { sch_ = sch; }
  void set_parent(Module * parent) { parent_ = parent; }

  //
  void apply_sensitivity(Sensitive s);

  //
  Scheduler * sch_{nullptr};
  Module * parent_{nullptr};
  std::string name_;

  Sensitive s_static_;
  Sensitive s_dynamic_;
};

} // namespace ccm::kernel

#endif
