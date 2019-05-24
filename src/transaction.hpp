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

#ifndef __SRC_TRANSACTION_HPP__
#define __SRC_TRANSACTION_HPP__

#include <deque>
#include <list>
#include <string>
#include "log.hpp"
#include "sim.hpp"
#include "utility.hpp"
#include "actor.hpp"
#include "options.hpp"
#ifndef ENABLE_JSON
#  include <nlohmann/json.hpp>
#  include <memory>
#endif

namespace ccm {

struct TransactionType {
#define TRANSACTION_TYPES(__func)               \
  __func(load)                                  \
  __func(store)                                 \
  __func(replacement)                           \
  __func(invalid)
  using type = uint8_t;
  enum : type {
#define __declare_type(__type) __type,
    TRANSACTION_TYPES(__declare_type)
#undef __declare_type
  };
  static const std::string to_string(type t);
  static type from_string(const std::string & s);
};

#define TRANSACTION_EVENT(__func) __func(Start) __func(End)

enum class TransactionEvent {
#define __declare_event(__name) __name,
  TRANSACTION_EVENT(__declare_event)
#undef __declare_event
};

const char *to_string(TransactionEvent event);

// clang-format off
#define TRANSACTION_FIELDS(__func)                                      \
  __func(TransactionType::type, type, TransactionType::invalid)         \
  __func(uint64_t, addr, 0)                                             \
  __func(std::size_t, tid, 0)
// clang-format on

struct Transaction : ccm::Poolable {
  Transaction() {}

#define __declare_getter_setter(__type, __name, __default) \
  using __name##_type = __type;                            \
  __type __name() const { return __name##_; }              \
  void set_##__name(const __type &__name) { __name##_ = __name; }

  TRANSACTION_FIELDS(__declare_getter_setter)
#undef __declare_getter_setter

  //
  void reset() { set_invalid(); }

 private:
  void set_invalid() {
#define __declare_defaults(__type, __name, __default) __name##_ = __default;

    TRANSACTION_FIELDS(__declare_defaults)
#undef __declare_defaults
  }

#define __declare_fields(__type, __name, __default) __type __name##_;

  TRANSACTION_FIELDS(__declare_fields)
#undef __declare_fields
};

std::string to_string(const Transaction &t);

// TODO: Deprecate
class TransactionFactory : Loggable {
 public:
  Transaction * construct();
 private:
  Pool<Transaction> pool_;
};

struct TransactionSource : Loggable {
#ifdef ENABLE_JSON
  static std::unique_ptr<TransactionSource> from_json(nlohmann::json & j);
#endif

  TransactionSource() {}
  virtual ~TransactionSource() {}

  //
  virtual bool is_active() const { return false; }

  //
  virtual bool get_transaction(TimeStamped<Transaction *> &ts) = 0;
  virtual void event(TransactionEvent event,
                     TimeStamped<const Transaction *> ts) = 0;
};

struct NullTransactionSource : TransactionSource {
#ifdef ENABLE_JSON
  static std::unique_ptr<TransactionSource> from_json(nlohmann::json & j);
#endif

  virtual bool get_transaction(TimeStamped<Transaction *> &ts) override;
  virtual void event(TransactionEvent event,
                     TimeStamped<const Transaction *> ts) override {}
};

struct ProgrammaticTransactionSource : TransactionSource {
#ifdef ENABLE_JSON
  static std::unique_ptr<TransactionSource> from_json(nlohmann::json & j);
#endif

  ProgrammaticTransactionSource() {}

  bool is_active() const override { return !pending_.empty(); }
  void add_transaction(TransactionType::type type, Time time, uint64_t addr);
  virtual bool get_transaction(TimeStamped<Transaction *> &ts) override;
  virtual void event(TransactionEvent event,
                     TimeStamped<const Transaction *> ts) override;
 private:
  Pool<Transaction> pool_;
  std::deque<TimeStamped<Transaction *> > pending_;
  std::list<Transaction *> in_flight_;
};

class TransactionQueueManager {
 public:
  using TSTransaction = TimeStamped<Transaction *>;
  
  explicit TransactionQueueManager();

  bool is_active() const;

  void push_back(const TSTransaction & t);
  void set_replacement(const TSTransaction & t);
  TSTransaction front() const;
  void pop_front();
  
 private:
  MinHeap<TSTransaction> q_;
  std::optional<TSTransaction> q_replacement_;
};

}  // namespace ccm

#endif
