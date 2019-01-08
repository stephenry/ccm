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

#include "utility.hpp"
#include "log.hpp"
#include <deque>
#include <string>
#include <list>

namespace ccm {

enum class TransactionType {
  Load,
  Store,
  Invalid
};

const char * to_string(TransactionType t);

#define TRANSACTION_FIELDS(__func)                              \
  __func(TransactionType, type, TransactionType::Invalid)       \
  __func(uint64_t, addr, 0)                                     \
  __func(std::size_t, tid, 0)
  
  
struct Transaction : ccm::Poolable {
  Transaction() {}

#define __declare_getter_setter(__type, __name, __default)              \
  using __name ## _type = __type;                                       \
  __type __name() const { return __name ## _; }                         \
  void set_ ## __name(const __type & __name) { __name ## _ = __name; }

  TRANSACTION_FIELDS(__declare_getter_setter)
#undef __declare_getter_setter

  //
  void reset() { set_invalid(); }

 private:
  void set_invalid() {
#define __declare_defaults(__type, __name, __default)   \
    __name ## _ = __default;

    TRANSACTION_FIELDS(__declare_defaults)
#undef __declare_defaults
  }

#define __declare_fields(__type, __name, __default)     \
  __type __name ## _;

  TRANSACTION_FIELDS(__declare_fields)
#undef __declare_fields
};

std::string to_string(const Transaction & t);

struct TransactionSource : Loggable {
  virtual ~TransactionSource() {}
  
  //
  std::size_t time() const override { return 0; }
  
  //
  virtual bool get_transaction(TimeStamped<Transaction *> & ts) = 0;

  //
  virtual void event_start(TimeStamped<Transaction *> ts) = 0;
  virtual void event_finish(TimeStamped<Transaction *> ts) = 0;
};

struct NullTransactionSource : TransactionSource {
  //
  virtual bool get_transaction(TimeStamped<Transaction *> & ts) override;

  //
  virtual void event_start(TimeStamped<Transaction *> ts) override {}
  virtual void event_finish(TimeStamped<Transaction *> ts) override {}
};
  
struct ProgrammaticTransactionSource : TransactionSource {
  //
  ProgrammaticTransactionSource() {}

  //
  void add_transaction(TransactionType type, std::size_t time, uint64_t addr);

  //
  virtual bool get_transaction(TimeStamped<Transaction *> & ts) override;

  //
  virtual void event_start(TimeStamped<Transaction *> ts) override;
  virtual void event_finish(TimeStamped<Transaction *> ts) override;

 private:
  Pool<Transaction> pool_;
  std::deque<TimeStamped<Transaction *> > pending_;
  std::list<Transaction *> in_flight_;
};

} // namespace ccm

#endif
