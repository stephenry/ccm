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

#ifndef __COHERENCE_HPP__
#define __COHERENCE_HPP__

#include "kernel/kernel.hpp"
#include "cache_model.hpp"
#include <memory>
#include <optional>

namespace ccm {

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

#define MESSAGE_CLASSES(__func)                 \
  __func(Load)                                  \
  __func(Store)                                 \
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
const char * to_string(MessageType t);

class CoherencyMessage : public kernel::Transaction {
  friend class CoherencyMessageBuilder;
  friend class GetSCoherencyMessageBuilder;
  friend class GetMCoherencyMessageBuilder;
  friend class PutSCoherencyMessageBuilder;
  friend class PutMCoherencyMessageBuilder;
  friend class FwdGetSCoherencyMessageBuilder;
  friend class FwdGetMCoherencyMessageBuilder;
  friend class InvCoherencyMessageBuilder;
  friend class DataCoherencyMessageBuilder;
  
 public:
  virtual ~CoherencyMessage() {}
  
  MessageType type() const { return type_; }
  std::size_t tid() const { return tid_; }
  bool is_ack() const { return is_ack_; }
  addr_t addr() const { return addr_; }
 private:
  MessageType type_;
  std::size_t tid_;
  addr_t addr_;
  bool is_ack_;
};

class CoherencyMessageBuilder {
 public:
  void set_tid(std::size_t id) { msg_->tid_ = id; }
  void set_is_ack(bool is_ack = true) { msg_->is_ack_ = is_ack; }
  void set_addr(addr_t a) { msg_->addr_ = a; }
 private:
  CoherencyMessage * msg_;
};

class LoadCoherencyMessage : public CoherencyMessage {
  friend class LoadCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class StoreCoherencyMessage : public CoherencyMessage {
 public:
  void reset() override {}
 private:
};

class GetSCoherencyMessage : public CoherencyMessage {
  friend class GetSCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class GetSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class GetSCoherencyMessageDirector;

  GetSCoherencyMessageBuilder(GetSCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::GetS;
  }
 public:
  ~GetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  GetSCoherencyMessage * msg() {
    GetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  GetSCoherencyMessage * msg_;
};

class GetSCoherencyMessageDirector {
 public:
  GetSCoherencyMessageBuilder builder() {
    return GetSCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<GetSCoherencyMessage> pool_;
};

class GetMCoherencyMessage : public CoherencyMessage {
  friend class GetMCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class GetMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class GetMCoherencyMessageDirector;

  GetMCoherencyMessageBuilder(GetMCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::GetM;
  }
 public:
  ~GetMCoherencyMessageBuilder() { if (msg_) msg_->release(); }
  GetMCoherencyMessage * msg() {
    GetMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
  }
 private:
  GetMCoherencyMessage * msg_;
};

class GetMCoherencyMessageDirector {
 public:
  GetMCoherencyMessageBuilder builder() {
    return GetMCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<GetMCoherencyMessage> pool_;
};

class PutSCoherencyMessage : public CoherencyMessage {
  friend class PutSCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class PutSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class PutSCoherencyMessageDirector;

  PutSCoherencyMessageBuilder(PutSCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::PutS;
  }
 public:
  ~PutSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutSCoherencyMessage * msg() {
    PutSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  PutSCoherencyMessage * msg_;
};

class PutSCoherencyMessageDirector {
 public:
  PutSCoherencyMessageBuilder builder() {
    return PutSCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<PutSCoherencyMessage> pool_;
};

class PutMCoherencyMessage : public CoherencyMessage {
  friend class PutMCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class PutMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class PutMCoherencyMessageDirector;

  PutMCoherencyMessageBuilder(PutMCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::PutM;
  }
 public:
  ~PutMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutMCoherencyMessage * msg() {
    PutMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  PutMCoherencyMessage * msg_;
};

class PutMCoherencyMessageDirector {
 public:
  PutMCoherencyMessageBuilder builder() {
    return PutMCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<PutMCoherencyMessage> pool_;
};

class FwdGetSCoherencyMessage : public CoherencyMessage {
  friend class FwdGetSCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class FwdGetSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class FwdGetSCoherencyMessageDirector;

  FwdGetSCoherencyMessageBuilder(FwdGetSCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::FwdGetS;
  }
 public:
  ~FwdGetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetSCoherencyMessage * msg() {
    FwdGetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  FwdGetSCoherencyMessage * msg_;
};

class FwdGetSCoherencyMessageDirector {
 public:
  FwdGetSCoherencyMessageBuilder builder() {
    return FwdGetSCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<FwdGetSCoherencyMessage> pool_;
};

class FwdGetMCoherencyMessage : public CoherencyMessage {
  friend class FwdGetMCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class FwdGetMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class FwdGetMCoherencyMessageDirector;

  FwdGetMCoherencyMessageBuilder(FwdGetMCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::FwdGetM;
  }
 public:
  ~FwdGetMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetMCoherencyMessage * msg() {
    FwdGetMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  FwdGetMCoherencyMessage * msg_;
};

class FwdGetMCoherencyMessageDirector {
 public:
  FwdGetMCoherencyMessageBuilder builder() {
    return FwdGetMCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<FwdGetMCoherencyMessage> pool_;
};

class InvCoherencyMessage : public CoherencyMessage {
  friend class InvCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class InvCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class InvCoherencyMessageDirector;

  InvCoherencyMessageBuilder(InvCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::Inv;
  }
 public:
  ~InvCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  InvCoherencyMessage * msg() {
    InvCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  InvCoherencyMessage * msg_;
};

class InvCoherencyMessageDirector {
 public:
  InvCoherencyMessageBuilder builder() {
    return InvCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<InvCoherencyMessage> pool_;
};

class DataCoherencyMessage : public CoherencyMessage {
  friend class DataCoherencyMessageBuilder;
 public:
  void reset() override {}
 private:
};

class DataCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class DataCoherencyMessageDirector;

  DataCoherencyMessageBuilder(DataCoherencyMessage * msg)
      : msg_(msg) {
    msg_->type_ = MessageType::Data;
  }
 public:
  ~DataCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  DataCoherencyMessage * msg() {
    DataCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
 private:
  DataCoherencyMessage * msg_;
};

class DataCoherencyMessageDirector {
 public:
  DataCoherencyMessageBuilder builder() {
    return DataCoherencyMessageBuilder{pool_.alloc()};
  }

 private:
  ccm::Pool<DataCoherencyMessage> pool_;
};

struct CoherentAgentOptions {
  Protocol protocol;
  CacheOptions cache_options;
  std::size_t max_in_flight_n{16};
};

struct CoherentAgentContext {
  ccm::kernel::Event wake_event;
};

#define RESPONSE_CLASSES(__func)                \
  __func(Hit)                                   \
  __func(Stall)                                 \
  __func(Blocked)

enum class ResponseType {
#define __declare_enum(e) e,
  RESPONSE_CLASSES(__declare_enum)
#undef __declare_enum
};
const char * to_string(ResponseType t);

struct CoherentAgentAction {
  ResponseType response;
  std::vector<CoherencyMessage *> msgs{nullptr};
  bool message_consumed{false};
};

class CoherentAgentModel {
 public:
  CoherentAgentModel(const CoherentAgentOptions & opts);
  
  virtual Protocol protocol() const = 0;
  virtual CoherentAgentAction apply(CoherentAgentContext & ctxt,
                                    CoherencyMessage * m) = 0;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts);

struct DirectoryOptions {
  Protocol protocol;
  CacheOptions cache_options;
};

struct DirectoryAction {
};

class DirectoryModel {
 public:
  DirectoryModel(const DirectoryOptions & opts);
  
  virtual Protocol protocol() const = 0;
  virtual DirectoryAction apply(CoherentAgentContext & ctxt,
                                CoherencyMessage * m) = 0;
};

std::unique_ptr<DirectoryModel> directory_factory(
    const DirectoryOptions & opts);

} // namespace ccm

#endif
