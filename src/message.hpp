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

namespace ccm {

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
  uint64_t addr() const { return addr_; }
  bool is_ack() const { return is_ack_; }

  void reset() {}

 private:
  void set_type(MessageType type) { type_ = type; }
  void set_src_id(std::size_t id) { src_id_ = id; }
  void set_dst_id(std::size_t id) { dst_id_ = id; }
  void set_tid(std::size_t tid) { tid_ = tid; }
  void set_addr(uint64_t addr) { addr_ = addr; }
  void set_is_ack(bool is_ack = true) { is_ack_ = is_ack; }

  MessageType type_;
  std::size_t src_id_;
  std::size_t dst_id_;
  std::size_t tid_;
  uint64_t addr_;
  bool is_ack_;
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

} // namespace ccm

#endif
