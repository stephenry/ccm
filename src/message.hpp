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

struct Transaction;

#define MESSAGE_CLASSES(__func)                 \
  __func(Request)                               \
  __func(Response)                              \
  __func(Data)

enum MessageClass {
#define __declare_class(__class)                \
  __class,
  MESSAGE_CLASSES(__declare_class)
#undef __declare_class
  CLASS_COUNT
};

#define MESSAGE_TYPES(__func)                   \
  __func(GetS, 1)                               \
  __func(GetM, 1)                               \
  __func(PutS, 1)                               \
  __func(PutM, 1)                               \
  __func(PutE, 1)                               \
  __func(PutO, 1)                               \
  __func(FwdGetS, 1)                            \
  __func(FwdGetM, 1)                            \
  __func(Inv, 1)                                \
  __func(AckCount, 1)                           \
  __func(Data, 4)

struct MessageType {
  using base_type = std::uint8_t;
  
  enum : base_type {
#define __declare_enum(__name, __cost) __name,
    MESSAGE_TYPES(__declare_enum)
#undef __declare_enum
    Invalid
  };

  static const char * to_string(base_type b);
  static int to_cost(base_type b);
};

struct Message : ccm::Poolable {
  friend class MessageBuilder;

  Message() { set_invalid(); }

#define MESSAGE_FIELDS(__func)                                  \
  __func(type, MessageType::base_type, MessageType::Invalid)    \
  __func(src_id, id_t, 1000)                                    \
  __func(dst_id, id_t, 1000)                                    \
  __func(fwd_id, id_t, 1000)                                    \
  __func(addr, uint64_t, 0)                                     \
  __func(is_ack, bool, false)                                   \
  __func(transaction, const Transaction *, nullptr)             \
  __func(ack_count, std::size_t, 0)                             \
  __func(is_exclusive, bool, false)                             \
  __func(was_owner, bool, false)

  MessageClass cls() const;
#define __declare_getter(__name, __type, __default)     \
  __type __name() const { return __name ## _; }
  MESSAGE_FIELDS(__declare_getter)
#undef __declare_getter

  void reset() { set_invalid(); }

 private:
  void set_invalid();

#define __declare_setter(__name, __type, __default)             \
  void set_ ## __name(__type __name) { __name ## _ = __name; }
  MESSAGE_FIELDS(__declare_setter)
#undef __declare_setter

#define __declare_field(__name, __type, __default)      \
  __type __name ## _;
  MESSAGE_FIELDS(__declare_field)
#undef __declare_field
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

#define __declare_setter(__name, __type, __default)     \
  void set_ ## __name(__type __name) { msg_->set_ ## __name(__name); }
  MESSAGE_FIELDS(__declare_setter)
#undef __declare_setter
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
