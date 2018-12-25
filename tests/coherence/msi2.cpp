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

#include <vector>
#include <deque>
#include <gtest/gtest.h>

enum class TransactionType {
  Load,
  Store
};

struct Transaction {
  Transaction(uint64_t addr, TransactionType type = TransactionType::Load)
      : addr_(addr), type_(type)
  {}

  //
  TransactionType type() const { return type_; }
  uint64_t addr() const { return addr_; }
  std::size_t tid() const { return tid_; }

  //
  void set_tid(std::size_t tid) { tid_ = tid; }

 private:
  TransactionType type_;
  uint64_t addr_;
  std::size_t tid_;
};

struct Message {

  std::size_t can_eval_at_time() const { return 0; }
  
  std::size_t src_id() const { return 0; }
  std::size_t dst_id() const { return 0; }

  void release() {}
};

class Action {
  friend class InterconnectModel;
  
 public:
  Action(std::size_t time, Message * message)
      : time_(time), message_(message)
  {}
  std::size_t time() const { return time_; }
  Message * message() const { return message_; }
 private:
  void set_time(std::size_t t) { time_ = t; }
  
  std::size_t time_;
  Message * message_;
};
bool operator<(const Action & lhs, const Action & rhs) {
  return (lhs.time() < lhs.time());
}

class Frontier {
 public:

  bool pending_transactions() const { return !v_.empty(); }
  
  void add_to_schedule(std::size_t t, Message * tn) {
    v_.push_back(Action{t, tn});
  }

  const std::vector<Action> & actions() const { return v_; }
  std::vector<Action> & actions() { return v_; }
  
  void finalize() {
    std::sort(v_.begin(), v_.end());
  }
  
  void clear() { v_.clear(); }

 private:
  std::vector<Action> v_;
};

class Schedule {

 public:
  Frontier get_frontier() { return Frontier{}; }
};

struct TransactionTable {

  TransactionTable()
      : is_fixed_(false)
  {}

  TransactionTable(std::size_t sz)
      : sz_(sz), is_fixed_(true)
  {}

  bool is_empty() const { return !ts_.empty(); }

  bool is_full() const {
    return !is_fixed_ || (ts_.size() == sz_);
  }

  std::size_t allocate(Transaction * t) {
    const std::size_t tid = ts_.size();
    
    t->set_tid(tid);
    ts_.push_back(t);

    return tid;
  }
 private:
  bool is_fixed_;
  std::size_t sz_;
  std::vector<Transaction *> ts_;
};

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
      : opts_(opts)
  {}
  virtual ~CoherentActor() {}

  std::size_t id() const { return opts_.id(); }

  virtual void apply(const Message * m) = 0;
  virtual bool eval(Frontier & f) = 0;
  virtual bool is_active() const = 0;
  
 private:
  const ActorOptions & opts_;
};

struct AgentOptions : ActorOptions {
  AgentOptions(std::size_t id)
      : ActorOptions(id)
  {}
};

struct Agent : CoherentActor {
  Agent(const AgentOptions & opts)
      : CoherentActor(opts), opts_(opts)
  {}

  void add_transaction(Transaction * t) {
    pending_transactions_.push_back(tt_.allocate(t));
  }

  bool can_accept() const {
    return !tt_.is_full();
  }

  void apply(const Message * m) override {
    pending_messages_.push_back(m);
  }
  
  bool eval(Frontier & f) override {
    return is_active();
  }
  
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
  std::deque<std::size_t> pending_transactions_;
  std::vector<const Message *> pending_messages_;
};

struct SnoopFilterOptions : ActorOptions {
  SnoopFilterOptions(std::size_t id)
      : ActorOptions(id)
  {}
};

struct SnoopFilter : CoherentActor {
  SnoopFilter(const SnoopFilterOptions & opts)
      : CoherentActor(opts), opts_(opts)
  {}

  void apply(const Message * m) override {
  }
  
  bool eval(Frontier & f) override {
    return false;
  }
  
  bool is_active() const override {
    return false;
  }
  const SnoopFilterOptions & opts_;
};

struct InterconnectModel {
  InterconnectModel() {}
  virtual ~InterconnectModel() {}
  
  void apply(Frontier & f) {
    for (Action & action : f.actions())
      update_time(action);
  }
 private:
  virtual std::size_t cost(std::size_t src_id, std::size_t dst_id) = 0;
  void update_time(Action & action) {
    const Message * m = action.message();
    action.set_time(action.time() + cost(m->src_id(), m->dst_id()));
  }
};

struct FixedLatencyInterconnectModel : InterconnectModel {
  FixedLatencyInterconnectModel(std::size_t latency)
      : latency_(latency)
  {}
 private:
  std::size_t cost(std::size_t src_id, std::size_t dst_id) override {
    // Fixed latency between all ARCS.
    return 10;
  }
  std::size_t latency_;
};

struct Sim {

  void add_actor(CoherentActor * a) {
    actors_.insert(std::make_pair(a->id(), a));
  }
  
  void run() {
    FixedLatencyInterconnectModel interconnect_model{10};
    Schedule s;

    Frontier f = s.get_frontier();
    while (has_active_actors()) {

      // Evaluate all actors in the platform
      //
      for (auto [t, actor] : actors_)
        actor->eval(f);

      // Forward all resultant transaction to target actors.
      //
      if (f.pending_transactions()) {
        interconnect_model.apply(f);
        f.finalize();
        for (const Action a : f.actions()) {
          Message * t = a.message();

          actors_[t->dst_id()]->apply(t);
          t->release();
        }
        f.clear();
      }
    }
    // All actors are inactive
  }

 private:
  bool has_active_actors() const {
    for (auto [t, actor] : actors_) {
      if (actor->is_active())
        return true;
    }
    return false;
  }
  
  std::map<std::size_t, CoherentActor *> actors_;
};

TEST(MSI, Load) {
  Sim s;

  std::vector<Agent *> agents_;
  for (std::size_t i = 0; i < 4; i++) {
    const AgentOptions opts(i);
    agents_.push_back(new Agent(opts));
    s.add_actor(agents_.back());
  }

  Transaction * t = new Transaction{0};
  agents_[0]->add_transaction(t);

  const SnoopFilterOptions opts(4);
  s.add_actor(new SnoopFilter(opts));
  s.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
