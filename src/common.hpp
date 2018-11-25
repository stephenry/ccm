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

#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <memory>
#include <vector>

namespace ccm {

  class Scheduler;

  class Module;
  using ModulePtr = std::unique_ptr<Module>;

  class Process;
  using ProcessPtr = std::unique_ptr<Process>;
  
  enum class SimState {
    Elaboration,
    Initialization,
    Running,
    Termination
  };

  enum RunMode : int { UntilTime, UntilExhaustion };

  struct RunOptions {
    RunOptions(std::size_t max_time)
      : run_mode(RunMode::UntilTime), max_time(max_time)
    {}
    RunOptions() {}
    bool can_run_at_time(std::size_t now) const {
      if (run_mode == RunMode::UntilExhaustion)
        return true;

      return (now <= max_time);
    }
  
    RunMode run_mode{RunMode::UntilExhaustion};
    std::size_t max_time;
  };

  class EventDescriptor;

  class EventHandle {
    friend class Scheduler;
  
    friend bool operator==(EventHandle const & a, EventHandle const & b);
    friend bool operator!=(EventHandle const & a, EventHandle const & b);

    EventHandle(EventDescriptor * ed) : ed_(ed) {}
  public:
    EventHandle() : ed_{nullptr} {}
    bool is_valid() const;
    void notify(std::size_t t = 0);
    void add_to_wait_set(Process * p);
    void remove_from_wait_set(Process *p);
  private:
    EventDescriptor *ed_{nullptr};
  };

  using EventOrList = std::vector<EventHandle>;

} // namespace ccm;

#endif
