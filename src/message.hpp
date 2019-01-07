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

#ifndef __SRC_MESSAGE_HPP__
#define __SRC_MESSAGE_HPP__

#include "utility.hpp"
#include "actors.hpp"
#include <string>

namespace ccm {

#define MESSAGE_CLASSES(__func)                 \
  __func(GetS)                                  \
  __func(GetM)                                  \
  __func(PutS)                                  \
  __func(PutM)                                  \
  __func(PutE)                                  \
  __func(PutO)                                  \
  __func(FwdGetS)                               \
  __func(FwdGetM)                               \
  __func(Inv)                                   \
  __func(AckCount)                              \
  __func(Data)

enum class MessageType {
#define __declare_enum(e) e,
  MESSAGE_CLASSES(__declare_enum)
#undef __declare_enum
  Invalid
};

const char * to_string(MessageType t);

struct Message : ccm::Poolable {
  friend class MessageBuilder;

  Message() : type_(MessageType::Invalid) { set_invalid(); }

  MessageType type() const { return type_; }
  std::size_t src_id() const { return src_id_; }
  std::size_t dst_id() const { return dst_id_; }
  uint64_t addr() const { return addr_; }
  bool is_ack() const { return is_ack_; }
  const Transaction * transaction() const { return transaction_; }
  std::size_t ack_count() const { return ack_count_; }
  bool is_exclusive() const { return is_exclusive_; }

  void reset() { set_invalid(); }

 private:
  void set_invalid();
  
  void set_type(MessageType type) { type_ = type; }
  void set_src_id(std::size_t id) { src_id_ = id; }
  void set_dst_id(std::size_t id) { dst_id_ = id; }
  void set_addr(uint64_t addr) { addr_ = addr; }
  void set_is_ack(bool is_ack = true) { is_ack_ = is_ack; }
  void set_transaction(const Transaction * t) { transaction_ = t; }
  void set_ack_count(std::size_t ack_count) { ack_count_ = ack_count; }
  void set_is_exclusive(bool is_exclusive) { is_exclusive_ = is_exclusive; }

  MessageType type_;
  std::size_t src_id_;
  std::size_t dst_id_;
  uint64_t addr_;
  bool is_ack_;
  const Transaction * transaction_;
  std::size_t ack_count_;
  bool is_exclusive_;
};

std::string to_string(const Message & m);

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
  void set_is_ack(bool is_ack = true) { msg_->set_is_ack(is_ack); }
  void set_transaction(const Transaction * t) { msg_->set_transaction(t); }
  void set_ack_count(std::size_t ack_count) { msg_->set_ack_count(ack_count); }
  void set_is_exclusive(bool is_exclusive) { msg_->set_is_exclusive(is_exclusive); }
 private:
  void set_src_id() { msg_->set_src_id(src_id_); }
  std::size_t src_id_;
  Message * msg_;
};

class MessageDirector {
 public:
  MessageDirector(const ActorOptions & opts);
  MessageBuilder builder();
 private:
  std::size_t src_id_;
  ccm::Pool<Message> pool_;
  const ActorOptions opts_;
};

} // namespace ccm

#endif
