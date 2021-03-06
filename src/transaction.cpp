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

#include "transaction.hpp"
#include <algorithm>
#include <unordered_map>
#include <exception>
#include "utility.hpp"

namespace ccm {

const std::string TransactionType::to_string(TransactionType::type t) {
  static const std::unordered_map<TransactionType::type, std::string> m{
#define __declare_mapping(__cmd) { __cmd, #__cmd },
    TRANSACTION_TYPES(__declare_mapping)
#undef __declare_mapping
  };
  auto it = m.find(t);
  if (it != m.end())
    return it->second;

  throw std::invalid_argument("Unknown TransactionType");
}

TransactionType::type TransactionType::from_string(const std::string & s) {
  static const std::unordered_map<std::string, TransactionType::type> m{
#define __declare_mapping(__cmd) { #__cmd, __cmd },
    TRANSACTION_TYPES(__declare_mapping)
#undef __declare_mapping
  };
  auto it = m.find(s);
  if (it != m.end())
    return it->second;

  throw std::invalid_argument("Unknown TransactionType");
}

std::string to_string(const Transaction &t) {
  using namespace std;

  StructRenderer sr;
  sr.add("type", to_string(t.type()));
  sr.add("addr", to_string(t.addr()));
  sr.add("tid", to_string(t.tid()));
  return sr.str();
}

const char *to_string(TransactionEvent event) {
  switch (event) {
#define __declare_string(__name) \
  case TransactionEvent::__name: \
    return #__name;              \
    break;
    TRANSACTION_EVENT(__declare_string)
#undef __declare_string
    default:
      return "Unknown";
      break;
  }
}

void Transaction::event(TransactionEvent event, const Time & time) const {
  owner_->event(this, event, time);
}

#ifdef ENABLE_JSON

std::unique_ptr<TransactionSource>
TransactionSource::from_json(nlohmann::json & j) {
  if (j["type"] == "null")
    return NullTransactionSource::from_json(j);
  if (j["type"] == "programmatic")
    return ProgrammaticTransactionSource::from_json(j["options"]);
  return nullptr;
}
#endif

Transaction * TransactionFactory::construct() {
  return pool_.alloc();
}
#ifdef ENABLE_JSON
std::unique_ptr<TransactionSource>
NullTransactionSource::from_json(nlohmann::json & j) {
  return std::make_unique<NullTransactionSource>();
}

#endif

void TransactionSource::initialize_transaction(Transaction * t) {
  t->owner_ = this;
}
#ifdef ENABLE_JSON

std::unique_ptr<TransactionSource>
ProgrammaticTransactionSource::from_json(nlohmann::json & j) {
  auto src = std::make_unique<ProgrammaticTransactionSource>();
  std::size_t tid{0};
  for (nlohmann::json & t : j["stimulus"]) {
    const TransactionType::type cmd{
      TransactionType::from_string(t["cmd"])};
    const Time time{t["time"]};
    const uint64_t addr{t["addr"]};
    src->add_transaction(cmd, time, addr, tid++);
  }
  return std::move(src);
}
#endif

void ProgrammaticTransactionSource::add_transaction(
    TransactionType::type type, Time time, uint64_t addr) {
  add_transaction(type, time, addr, tid_++);
}

void ProgrammaticTransactionSource::add_transaction(
    TransactionType::type type, Time time, uint64_t addr, std::size_t tid) {
  Transaction *t = pool_.alloc();

  TransactionSource::initialize_transaction(t);
  t->set_type(type);
  t->set_addr(addr);
  t->set_tid(tid);
  
  pending_.push_back(TimeStamped<Transaction *>{time, t});
}

bool ProgrammaticTransactionSource::get_transaction(
    TimeStamped<Transaction *> &ts) {
  if (pending_.size() == 0) return false;

  ts = pending_.front();
  pending_.pop_front();
  return true;
}

void ProgrammaticTransactionSource::event(
    const Transaction * t, TransactionEvent event, const Time & time) {
  switch (event) {
    case TransactionEvent::Start:
      break;
    case TransactionEvent::End:
      // Remove const-ness as we are returning the transaction back to
      // the pool and because we allow the pool to reset the state of
      // the object. This is more convenient overall as we can retain
      // constness throughout the user code (whereas the non-costness
      // would have to propagate).
      //
      const_cast<Transaction *>(t)->release();
      break;
  }
}

TransactionQueueManager::TransactionQueueManager() {}

bool TransactionQueueManager::is_active() const { return !q_.empty(); }

void TransactionQueueManager::push_back(const TSTransaction & t) { q_.push(t); }

TransactionQueueManager::TSTransaction
TransactionQueueManager::front() const { return q_replacement_.value_or(q_.top()); }

void TransactionQueueManager::set_replacement(const TSTransaction & t) {
  q_replacement_ = t;
}

void TransactionQueueManager::pop_front() {
  if (q_replacement_) {
    q_replacement_.reset();
  } else {
    q_.pop();
  }
}

}  // namespace ccm
