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

#ifndef __SRC_SIM_HPP__
#define __SRC_SIM_HPP__

#include <map>
#include <vector>
#include "types.hpp"
#include "utility.hpp"
#include "platform.hpp"
#include "log.hpp"

namespace ccm {

class Message;
class Transaction;
class CoherentActor;
class CoherenceProtocolValidator;
class InterconnectModel;

std::string to_string(const Time &t);

class Cursor {
  friend class Epoch;

  Cursor(Time time, Time step) : time_(time), step_(step) {}

 public:
  void set_time(Time time) { time_ = time; }
  void set_time_to_greater(Time newtime);
  void advance(std::size_t steps);

  Time time() const { return time_; }
  Time step() const { return step_; }

 private:
  Time time_, step_;
};

struct Epoch {
  Epoch(Time start, Time duration, Time step);

  void set_cursor(Time cursor) { cursor_ = cursor; }

  bool in_interval(Time t) const;
  Epoch advance() const;
  void step();

  Cursor cursor() const { return Cursor{start(), step()}; }
  Time start() const { return start_; }
  Time end() const { return start_ + duration_; }
  Time duration() const { return duration_; }
  Time step() const { return step_; }

 private:
  Time start_, duration_, step_, cursor_;
};

std::string to_string(const Epoch &epoch);

template <typename T>
class TimeStamped {
 public:
  TimeStamped() {}
  TimeStamped(Time time, const T &t) : t_(t), time_(time) {}

  Time time() const { return time_; }
  T t() const { return t_; }

  void set_time(Time time) { time_ = time; }

 private:
  Time time_;
  T t_;
};

template <typename T>
std::string to_string(const TimeStamped<T> &ts) {
  using namespace std;

  StructRenderer sr;
  sr.add("time", to_string(ts.time()));
  sr.add("t", to_string(*ts.t()));
  return sr.str();
}

template <typename T>
bool operator<(const TimeStamped<T> &lhs, const TimeStamped<T> &rhs) {
  return (lhs.time() < rhs.time());
}

template <typename T>
bool operator>(const TimeStamped<T> &lhs, const TimeStamped<T> &rhs) {
  return (lhs.time() > rhs.time());
}

struct Context {
  friend class ExecutionContext;

  Context(const Epoch &epoch) : epoch_(epoch) {}

  Epoch epoch() const { return epoch_; }
  void emit_message(TimeStamped<Message *> msg);
  const std::vector<TimeStamped<Message *>> & msgs() const { return msgs_; }

 private:
  std::vector<TimeStamped<Message *> > msgs_;
  Epoch epoch_;
};

class RunOptions {
  enum class Terminate { OnExhaustion, AfterTime };

 public:
  RunOptions() : terminate_(Terminate::OnExhaustion) {}
  RunOptions(Time time) : terminate_(Terminate::AfterTime), final_(time) {}

  Terminate terminate() const { return terminate_; }
  bool has_completed(Time current) const;

 private:
  Terminate terminate_;
  Time final_;
};

struct Sim {
  friend class Builder;
  
  Sim();
  ~Sim();

  const Platform & platform() const { return platform_; }

  void add_actor(std::unique_ptr<CoherentActor> && actor);
  void add_interconnect(std::unique_ptr<InterconnectModel> && inter);

  void run(const RunOptions &run_options = RunOptions{});

 private:
  void set_time(Time time) { time_ = time; }
  Time time() const { return time_; }
  bool has_active_actors() const;

  Platform platform_;
  Time time_;
  std::map<std::size_t, std::unique_ptr<CoherentActor> > actors_;
  std::unique_ptr<InterconnectModel> interconnect_;
  Logger logger_;
  std::size_t epoch_period_{100};
  std::size_t epoch_step_{10};
};

}  // namespace ccm

#endif
