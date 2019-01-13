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

#include "sim.hpp"
#include "message.hpp"
#include "interconnect.hpp"

namespace ccm {

std::string to_string(const Time & t) {
  std::stringstream ss;
  ss << static_cast<unsigned>(t);
  return ss.str();
}

void Cursor::advance(std::size_t steps) {
  set_time(time() + (step() * steps));
}

Epoch::Epoch(Time start, Time duration, Time step)
  : start_(start), duration_(duration), step_(step), cursor_(start)
{}

bool Epoch::in_interval(Time t) const {
  return (t < end());
}

Epoch Epoch::advance() const {
  return Epoch{end(), duration(), step()};
}

std::string to_string(const Epoch & epoch) {
  using namespace std;
  
  StructRenderer sr;
  sr.add("start", to_string(epoch.start()));
  sr.add("end", to_string(epoch.end()));
  sr.add("step", to_string(epoch.step()));
  return sr.str();
}

QueueEntry::QueueEntry()
  : type_(QueueEntryType::Invalid)
{}
  
QueueEntry::QueueEntry(message_queue_type * msgq)
  : type_(QueueEntryType::Message), msgq_(msgq)
{}

QueueEntry::QueueEntry(transaction_queue_type * trnq)
  : type_(QueueEntryType::Transaction), trnq_(trnq)
{}

std::string to_string(const QueueEntry & eq) {
  using namespace std;
  
  StructRenderer sr;
  sr.add("type", (eq.type() == QueueEntryType::Message)
          ? "Message" : "Transaction");
  sr.add("time", to_string(eq.time()));
  switch (eq.type()) {
  case QueueEntryType::Message:
    sr.add("msg", to_string(eq.as_msg()));
    break;

  case QueueEntryType::Transaction:
    sr.add("msg", to_string(eq.as_trn()));
    break;

  default:;
  }
  return sr.str();
}

bool operator<(const QueueEntry & lhs, const QueueEntry & rhs) {
  return lhs.time() < rhs.time();
}

bool operator>(const QueueEntry & lhs, const QueueEntry & rhs) {
  return lhs.time() > rhs.time();
}

Time QueueEntry::time() const {
  Time ret{0};
  switch (type_) {
  case QueueEntryType::Message: {
    typename message_queue_type::value_type ts = msgq_->top();
    ret = ts.time();
  } break;

  case QueueEntryType::Transaction: {
    typename transaction_queue_type::value_type ts = trnq_->top();
    ret = ts.time();
  } break;

  default:
    // TODO: Error
    ;
  }
  return ret;
}
  
typename message_queue_type::value_type QueueEntry::as_msg() const {
  return msgq_->top();
}

typename transaction_queue_type::value_type QueueEntry::as_trn() const {
  return trnq_->top();
}

QueueManager::QueueManager() {
  messages_.resize(CLASS_COUNT);
}

bool QueueManager::empty() const {
  if (!transactions_.empty())
    return false;
  
  for (std::size_t i = 0; i < CLASS_COUNT; i++)
    if (!messages_[i].empty())
      return false;
  
  return true;
}

void QueueManager::push(TimeStamped<Message *> ts) {
  messages_[ts.t()->cls()].push(ts);
}
  
void QueueManager::push(TimeStamped<Transaction *> ts) {
  transactions_.push(ts);
}

QueueEntry QueueManager::next() {
  MinHeap<QueueEntry> pq;

  if (!transactions_.empty()) {
    const Transaction * trn = transactions_.top().t();
    if (!tac_ || tac_->can_be_issued(trn))
      pq.push(QueueEntry{std::addressof(transactions_)});
  }

  for (std::size_t i = 0; i < CLASS_COUNT; i++) {
    if (!messages_[i].empty()) {
      const Message * msg = messages_[i].top().t();
      if (!mac_ || mac_->can_be_issued(msg))
        pq.push(QueueEntry{std::addressof(messages_[i])});
    }
  }
  return !pq.empty() ? pq.top() : QueueEntry{};
}

void QueueEntry::consume() const {
  switch (type()) {
  case QueueEntryType::Message: msgq_->pop(); break;
  case QueueEntryType::Transaction: trnq_->pop(); break;
  default: ;
  }
}

void Context::emit_message(TimeStamped<Message *> msg) {
  msgs_.push_back(msg);
}

void Sim::add_actor(CoherentActor * a) {
  actors_.insert(std::make_pair(a->id(), a));
}

void Sim::run() {
  FixedLatencyInterconnectModel interconnect_model{10};

  Epoch current_epoch{0, 20, 10};
  do {
    Context ctxt{current_epoch};
    for (auto [t, actor] : actors_)
      actor->eval(ctxt);

    for (TimeStamped<Message *> ts : ctxt.msgs_) {
      interconnect_model.apply(ts);
        
      const Message * msg = ts.t();
      actors_[msg->dst_id()]->apply(ts);
    }
    current_epoch = current_epoch.advance();
  } while (has_active_actors());
}

bool Sim::has_active_actors() const {
  for (auto [t, actor] : actors_) {
    if (actor->is_active())
      return true;
  }
  return false;
}

} // namespace ccm
