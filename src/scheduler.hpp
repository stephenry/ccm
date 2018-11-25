#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

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

#include "src/event.hpp"

#include <vector>
#include <map>
#include <memory>

namespace ccm {

class Process;
class Scheduler;
class Module;

struct FrontierTask {
  virtual bool is_nop() const { return true; }
  virtual void apply(Scheduler * sch) {};
  virtual ~FrontierTask() {}
  std::size_t time() const { return t_; }
 protected:
  std::size_t t_;
};
using FrontierTaskPtr = std::unique_ptr<FrontierTask>;

class Frontier {
 public:
  Frontier();

  //
  bool work_remains() const { return !f_.empty(); }
  void add_work(std::size_t t, FrontierTaskPtr p) { f_[t].push_back(std::move(p)); }

  //
  std::size_t next_time() const { return f_.begin()->first; }
  std::vector<FrontierTaskPtr> & next() { return f_.begin()->second; }
  void advance() { f_.erase(f_.begin()); }
  
 private:
  std::map<std::size_t, std::vector<FrontierTaskPtr> > f_;
};

enum SimState : int {
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

class Scheduler {
  friend class EventDescriptor;
  friend class EventOrDescriptor;
  friend class WakeProcessTask;
  
 public:
  Scheduler();
  ~Scheduler();

  //
  SimState state() const { return sim_state_; }
  std::size_t now() const { return now_; }
  std::size_t delta() const { return delta_; }
    
  //
  void run(RunOptions const & run_options = RunOptions());

  //
  EventHandle create_event();
  EventHandle create_event(EventOrList const & e);

  //
  void add_process (Process * p);
  void add_process (Process & p) { add_process(std::addressof(p)); }


  //
  void set_top (Module * top) { top_ = top; }

 private:
  void set_state(SimState sim_state) { sim_state_ = sim_state; };
  void do_next_delta();

  //
  void add_task_wake_after(Process * p, std::size_t time = 0);
  void add_task_notify_after(EventHandle h, std::size_t time = 0);
  
  //
  std::vector<EventDescriptorPtr> events_;

  //
  std::vector<Process *> current_delta_, next_delta_;

  //
  std::vector<Process *> reaped_processes_;

  //
  Frontier frontier_;

  //
  SimState sim_state_{SimState::Initialization};

  //
  Module * top_{nullptr};

  //
  std::size_t delta_{0};
  std::size_t now_{0};
};

} // namespace ccm

#endif
