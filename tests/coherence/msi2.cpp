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

#include "ccm.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <iostream>
#include <gtest/gtest.h>

enum class TransactionType {
  Load,
  Store
};

enum class TransactionEvents {
  Enqueue,
  Issue
};

class Transaction {

  struct Event {
    TransactionEvents e;
    std::size_t t;
  };

 public:
  Transaction(uint64_t addr, TransactionType type = TransactionType::Load)
      : addr_(addr), type_(type)
  {}

  //
  TransactionType type() const { return type_; }
  uint64_t addr() const { return addr_; }
  std::size_t tid() const { return tid_; }
  void add_event(TransactionEvents e, std::size_t t) {
    e_.push_back(Event{e, t});
  }

  //
  void set_tid(std::size_t tid) { tid_ = tid; }

 private:
  TransactionType type_;
  uint64_t addr_;
  std::size_t tid_;
  std::vector<Event> e_;
};

#define MESSAGE_CLASSES(__func)                 \
  __func(GetS)                                  \
  __func(GetM)                                  \
  __func(PutS)                                  \
  __func(PutM)                                  \
  __func(FwdGetS)                               \
  __func(FwdGetM)                               \
  __func(Inv)                                   \
  __func(Data)

enum class MessageType {
#define __declare_enum(e) e,
  MESSAGE_CLASSES(__declare_enum)
#undef __declare_enum
  Invalid
};

struct Message : ccm::Poolable {
  friend class MessageBuilder;

  Message() : type_(MessageType::Invalid) {}
  
  MessageType type() const { return type_; }
  std::size_t src_id() const { return src_id_; }
  std::size_t dst_id() const { return dst_id_; }
  std::size_t tid() const { return tid_; }

  void reset() {}

 private:
  void set_type(MessageType type) { type_ = type; }
  void set_src_id(std::size_t id) { src_id_ = id; }
  void set_dst_id(std::size_t id) { dst_id_ = id; }
  void set_tid(std::size_t tid) { tid_ = tid; }

  MessageType type_;
  std::size_t src_id_;
  std::size_t dst_id_;
  std::size_t tid_;
};

template<typename T>
class TimeStamped {
 public:
  TimeStamped() {}
  TimeStamped(std::size_t time, T & t) : t_(t), time_(time) {}
  
  std::size_t time() const { return time_; }
  T t() const { return t_; }

  void set_time(std::size_t time) { time_ = time; }
 private:
  std::size_t time_;
  T t_;
};

template<typename T>
bool operator<(const TimeStamped<T> & lhs, const TimeStamped<T> & rhs) {
  return (lhs.time() < lhs.time());
}

template<typename T>
auto make_time_stamped(std::size_t time, T & t) -> TimeStamped<T> {
  return TimeStamped<T>(time, t);
}


template<typename T>
class Heap {
 public:
  Heap() : is_heap_(true) {}

  std::vector<T> & ts() { return ts_; }
  const std::vector<T> & ts() const { return ts_; }
  
  void clear() {
    ts_.clear();
    is_heap_ = true;
  }

  bool empty() const {
    return ts_.empty();
  }

  bool peak_head(T & t) {
    bool ret = false;
    if (!empty()) {

      if (!is_heap_)
        heapify();

      t = ts_.front();
    }
    return ret;
  }

  void push(const T & t) {
    ts_.push_back(t);
    is_heap_ = false;
  }

  bool pop(T & t) {
    bool ret = false;
    if (!empty()) {

      if (!is_heap_)
        heapify();
        
      t = ts_.front();
      ts_.erase(ts_.begin());
      ret = true;
    }
    return ret;
  }
  
  void heapify() {
    std::make_heap(ts_.begin(), ts_.end());
    is_heap_ = true;
  }
 private:
  bool is_heap_;
  std::vector<T> ts_;
};

class MessageBuilder {
 public:
  MessageBuilder(Message * msg, std::size_t src_id)
      : msg_(msg), src_id_(src_id) {
    set_src_id();
  }
  ~MessageBuilder() { if (msg_) msg_->release(); }

  Message * msg() {
    Message * m{nullptr};
    std::swap(m, msg_);
    return m;
  }
  
  void set_type(MessageType type) { msg_->set_type(type); }
  void set_dst_id(std::size_t id) { msg_->set_dst_id(id); }
  void set_tid(std::size_t tid) { msg_->set_tid(tid); }
 private:
  void set_src_id() { msg_->set_src_id(src_id_); }
  std::size_t src_id_;
  Message * msg_;
};

class MessageDirector {
 public:
  MessageDirector(std::size_t src_id) : src_id_(src_id)
  {}
      
  MessageBuilder builder() {
    return MessageBuilder{pool_.alloc(), src_id_};
  }
 private:
  std::size_t src_id_;
  ccm::Pool<Message> pool_;
};

class Frontier {
 public:

  std::vector<TimeStamped<const Message *>> & ts() { return msgs_.ts(); }

  void add_to_frontier(std::size_t t, const Message * msg) {
    msgs_.push(make_time_stamped(t, msg));
  }

  bool pending_transactions() const {
    return !msgs_.empty();
  }

  bool pop(TimeStamped<const Message *> & msg) {
    return msgs_.pop(msg);
  }

  void clear() { msgs_.clear(); }

  void heapify() {
    msgs_.heapify();
  }

 private:
  Heap<TimeStamped<const Message *> > msgs_;
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

  void add_transaction(std::size_t time, Transaction * t) {
    pending_transactions_.push(make_time_stamped(time, t));
  }

  bool can_accept() const {
    return !tt_.is_full();
  }

  void apply(std::size_t t, const Message * m) override {
    pending_messages_.push_back(m);
  }
  
  bool eval(Frontier & f) override {
    if (!pending_transactions_.empty()) {

      TimeStamped<Transaction *> head;
      while (pending_transactions_.pop(head)) {
        set_time(head.time());
        
        MessageBuilder b = msgd_.builder();
        b.set_type(MessageType::GetS);
        b.set_dst_id(4);
        b.set_tid(1);
        f.add_to_frontier(1 + head.time(), b.msg());
        
        std::cout << time() << " SnoopActor: Sending something\n";
      }
    }
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
  Heap<TimeStamped<Transaction *> > pending_transactions_;
  std::vector<const Message *> pending_messages_;
  MessageDirector msgd_;
};

struct SnoopFilterOptions : ActorOptions {
  SnoopFilterOptions(std::size_t id)
      : ActorOptions(id)
  {}
};

struct SnoopFilter : CoherentActor {
  SnoopFilter(const SnoopFilterOptions & opts)
      : CoherentActor(opts), opts_(opts), msgd_(opts.id())
  {}

  void apply(std::size_t t, const Message * m) override {
    pending_messages_.push(make_time_stamped(t, m));
  }
  
  bool eval(Frontier & f) override {
    if (!pending_messages_.empty()) {

      TimeStamped<const Message *> head;
      while (pending_messages_.pop(head)) {
        set_time(head.time());
      
        std::cout << time() << " SnoopFilter: Received something\n";
      }
    }
    return is_active();
  }
  
  bool is_active() const override {
    return !pending_messages_.empty();
  }
 private:
  const SnoopFilterOptions & opts_;
  Heap<TimeStamped<const Message *> > pending_messages_;
  MessageDirector msgd_;
};

struct InterconnectModel {
  InterconnectModel() {}
  virtual ~InterconnectModel() {}
  
  void apply(Frontier & f) {
    for (TimeStamped<const Message *> & ts : f.ts())
      update_time(ts);
    f.heapify();
  }
 private:
  virtual std::size_t cost(std::size_t src_id, std::size_t dst_id) = 0;
  void update_time(TimeStamped<const Message *> & ts) {
    const Message * m = ts.t();
    ts.set_time(ts.time() + cost(m->src_id(), m->dst_id()));
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

  Sim() : time_(0) {}

  void add_actor(CoherentActor * a) {
    actors_.insert(std::make_pair(a->id(), a));
  }
  
  void run() {
    FixedLatencyInterconnectModel interconnect_model{10};

    Frontier f;
    while (has_active_actors()) {

      // Evaluate all actors in the platform
      //
      for (auto [t, actor] : actors_)
        actor->eval(f);

      // Forward all resultant transaction to target actors.
      //
      if (f.pending_transactions()) {
        interconnect_model.apply(f);
 
        TimeStamped<const Message *> head;
        while (f.pop(head)) {
          set_time(head.time());
          
          const Message * t = head.t();
          actors_[t->dst_id()]->apply(time(), t);
          t->release();
        }
        f.clear();
      }
    }
    // All actors are inactive
  }

 private:
  void set_time(std::size_t time) { time_ = time; }
  std::size_t time() const { return time_; }
  
  bool has_active_actors() const {
    for (auto [t, actor] : actors_) {
      if (actor->is_active())
        return true;
    }
    return false;
  }

  std::size_t time_;
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
  t->add_event(TransactionEvents::Enqueue, 10);
  agents_[0]->add_transaction(10, t);

  const SnoopFilterOptions opts(4);
  s.add_actor(new SnoopFilter(opts));
  s.run();
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
