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

#include <vector>

namespace ccm {

enum class TransactionType {
  Load,
  Store
};

class Transaction {
 public:
  using tid_type = std::size_t;

  
  Transaction(uint64_t addr, TransactionType type = TransactionType::Load)
      : addr_(addr), type_(type)
  {}

  //
  TransactionType type() const { return type_; }
  uint64_t addr() const { return addr_; }
  tid_type tid() const { return tid_; }

  //
  void set_tid(std::size_t tid) { tid_ = tid; }

 private:
  TransactionType type_;
  uint64_t addr_;
  tid_type tid_;
};

struct TransactionTable {

  TransactionTable()
      : is_fixed_(false)
  {}

  TransactionTable(std::size_t sz)
      : sz_(sz), is_fixed_(true)
  {}

  bool is_empty() const { return ts_.empty(); }
  bool is_full() const { return !is_fixed_ || (ts_.size() == sz_); }

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

} // namespace ccm

#endif
