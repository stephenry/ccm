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

#ifndef __SCHEDULER_HPP__
#define __SCHEDULER_HPP__

#include "module.hpp"

#include <vector>
#include <map>
#include <memory>

namespace ccm::kernel {

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

struct ElaborationState {
  Scheduler * sch;
};
  
enum class SimState {
  Elaboration,
  Initialization,
  Running,
  Termination
};

struct Task {
  virtual ~Task() {}

  //
  virtual std::string what() const = 0;
  virtual std::size_t time() const = 0;

  //
  virtual void apply() = 0;
};

class Frontier {
 public:

  //
  Frontier();

  //
  bool work_remains() const;
  std::size_t next_time() const;

  //
  std::vector<std::unique_ptr<Task> > & next();
  void add_work(std::size_t t, std::unique_ptr<Task> && p);
  void advance();

  //
  void set_logger(Logger * logger) { logger_ = logger; }
  
 private:
  //
  template<typename ... ARGS>
  void debug(ARGS && ... args) {
    if (logger_) { logger_->log(LogLevel::Debug, std::forward<ARGS>(args)...); }
  }
  
  std::map<std::size_t, std::vector<std::unique_ptr<Task> > > f_;
  Logger * logger_;
};

class Scheduler {
  friend class Module;
  friend class Process;
  friend class NormalEventContext;
  friend class WakeProcessAtTimeTask;
  friend class EventContext;

  static const std::size_t DELTA_MAX = 1000;
  
 public:
  //
  Scheduler();
  ~Scheduler();

  //
  SimState state() const { return sim_state_; }
  std::size_t now() const { return now_; }
  std::size_t delta() const { return delta_; }

  //
  void run(RunOptions const & run_options = RunOptions());

  //
  void set_top (Module * ptr);
  void set_logger (Logger * logger) { logger_ = logger; frontier_.set_logger(logger); }

 private:

  //
  void add_process (Process * p);
  void add_frontier_task(std::unique_ptr<Task> && t);
  void add_process_next_delta(Process * p);
  void set_state(SimState sim_state) { sim_state_ = sim_state; };

  //
  std::string prefix() const;
  template<typename ... ARGS>
  void debug(ARGS && ... args) {
    if (logger_) { logger_->log(LogLevel::Debug, prefix(), std::forward<ARGS>(args)...); }
  }
  
  //
  std::vector<Process *> current_delta_, next_delta_;
  Frontier frontier_;
  SimState sim_state_{SimState::Initialization};
  Module * top_{nullptr};
  Logger * logger_{nullptr};

  //
  std::size_t delta_{0};
  std::size_t now_{0};
};

} // namespace ccm::kernel

#endif
