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

#ifndef __SRC_ACTORS_HPP__
#define __SRC_ACTORS_HPP__

#include "log.hpp"
#include "transaction.hpp"
#include "message.hpp"

namespace ccm {

class Frontier;

struct ActorOptions {
  ActorOptions(std::size_t id)
      : id_(id)
  {}
  std::size_t id() const { return id_; }
 private:
  std::size_t id_;
};

struct CoherentActor {
  CoherentActor(const ActorOptions & opts)
      : opts_(opts), time_(0)
  {}
  virtual ~CoherentActor() {}

  std::size_t time() const { return time_; }
  std::size_t id() const { return opts_.id(); }

  virtual void apply(std::size_t t, const Message * m) = 0;
  virtual bool eval(Frontier & f) = 0;
  virtual bool is_active() const = 0;

  void set_time(std::size_t time) { time_ = time; }
  
 private:
  std::size_t time_;
  const ActorOptions & opts_;
};

struct AgentOptions : ActorOptions {
  AgentOptions(std::size_t id)
      : ActorOptions(id)
  {}
};

struct Agent : CoherentActor {
  Agent(const AgentOptions & opts)
      : CoherentActor(opts), opts_(opts), msgd_(opts.id())
  {}

  void add_transaction(std::size_t time, Transaction * t);

  bool can_accept() const {
    return !tt_.is_full();
  }

  void apply(std::size_t t, const Message * m) override {
    pending_messages_.push_back(m);
  }
  
  bool eval(Frontier & f) override;
  
  bool is_active() const override {
    // Agent is active if there are pending transction in the
    // Transaction Table, or if there are transaction awaiting to be
    // issued.
    //
    return (!tt_.is_empty() || !pending_transactions_.empty());
  }
  
 private:
  const AgentOptions & opts_;
  TransactionTable tt_;
  Heap<TimeStamped<Transaction *> > pending_transactions_;
  std::vector<const Message *> pending_messages_;
  MessageDirector msgd_;
};

struct SnoopFilterActorOptions : ActorOptions {
  SnoopFilterActorOptions(std::size_t id)
      : ActorOptions(id)
  {}
};

struct SnoopFilter : CoherentActor {
  SnoopFilter(const SnoopFilterActorOptions & opts)
      : CoherentActor(opts), opts_(opts), msgd_(opts.id())
  {}

  void apply(std::size_t t, const Message * m) override;
  
  bool eval(Frontier & f) override;
  
  bool is_active() const override {
    return !pending_messages_.empty();
  }
 private:
  const SnoopFilterActorOptions & opts_;
  Heap<TimeStamped<const Message *> > pending_messages_;
  MessageDirector msgd_;
};

} // namespace ccm

#endif
