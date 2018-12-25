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

#include "cache_model.hpp"
#include "kernel/kernel.hpp"
#include <memory>
#include <optional>

namespace ccm {

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

#define TRANSACTION_CLASSES(__func)              \
  __func(Load)                                   \
  __func(Store)

enum class TransactionType {
#define __declare_enum(e) e,
  TRANSACTION_CLASSES(__declare_enum)
#undef __declare_enum
  Invalid
};
const char * to_string(TransactionType t);

struct Transaction {
 public:
  static Transaction make_load(addr_t a) {
    return Transaction{TransactionType::Load, a};
  }
  static Transaction make_store(addr_t a) {
    return Transaction{TransactionType::Store, a};
  }

  Transaction(TransactionType type, addr_t addr)
      : type_(type), addr_(addr)
  {}
  
  TransactionType type() const { return type_; }
  addr_t addr() const { return addr_; }
 private:
  TransactionType type_;
  addr_t addr_;
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
const char * to_string(MessageType t);

class CoherencyMessage : public kernel::Transaction {
  friend class CoherencyMessageBuilder;
  friend class LoadCoherencyMessageBuilder;
  friend class StoreCoherencyMessageBuilder;
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
  void set_tid(std::size_t tid) { tid_ = tid; }
  
  MessageType type_;
  std::size_t tid_;
  addr_t addr_;
  bool is_ack_;
};

class CoherencyMessageBuilder : public kernel::TransactionBuilder {
 public:
  CoherencyMessageBuilder(CoherencyMessage * msg)
      : msg_(msg), TransactionBuilder(msg)
  {}
  void set_is_ack(bool is_ack = true) { msg_->is_ack_ = is_ack; }
  void set_addr(addr_t a) { msg_->addr_ = a; }
  void set_tid(std::size_t tid) { msg_->set_tid(tid); }
 private:
  CoherencyMessage * msg_;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::GetS;
  }
 public:
  ~GetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  GetSCoherencyMessage * msg() {
    GetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::PutS;
  }
 public:
  ~PutSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutSCoherencyMessage * msg() {
    PutSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::PutM;
  }
 public:
  ~PutMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutMCoherencyMessage * msg() {
    PutMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::FwdGetS;
  }
 public:
  ~FwdGetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetSCoherencyMessage * msg() {
    FwdGetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::FwdGetM;
  }
 public:
  ~FwdGetMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetMCoherencyMessage * msg() {
    FwdGetMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::Inv;
  }
 public:
  ~InvCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  InvCoherencyMessage * msg() {
    InvCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
  std::size_t ack_count() const { return ack_count_; }
  void reset() override {}
 private:
  std::size_t ack_count_;
};

class DataCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class DataCoherencyMessageDirector;

  DataCoherencyMessageBuilder(DataCoherencyMessage * msg)
      : msg_(msg), CoherencyMessageBuilder(msg) {
    msg_->type_ = MessageType::Data;
  }
 public:
  ~DataCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  void set_ack_count(std::size_t ack_count) { msg_->ack_count_ = ack_count; }
  DataCoherencyMessage * msg() {
    DataCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
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
  CoherentAgentOptions(std::size_t id)
      : id_(id)
  {}

  std::size_t id() const { return id_; }
  
  CacheOptions cache_options;

  std::size_t id_;
  
  std::size_t max_in_flight_n{16};
};

#define COHERENT_ACTOR_RESULT(__func)      \
  __func(Advances)                              \
  __func(BlockedOnProtocol)                     \
  __func(TagsExhausted)

enum class CoherentActorResultStatus {
#define __declare_enum(e) e,
  COHERENT_ACTOR_RESULT(__declare_enum)
#undef __declare_enum
};

struct CoherentActorResult {
  CoherentActorResult()
  {}

  CoherentActorResultStatus status() const { return status_; }

  void set_status(CoherentActorResultStatus status) { status_ = status; }
  void add_msg(CoherencyMessage * m) { msgs_.push_back(m); }

  std::vector<CoherencyMessage *> msgs() const { return msgs_; }

 private:
  CoherentActorResultStatus status_;
  std::vector<CoherencyMessage *> msgs_;
};

class CoherentActorBase {
 public:
  virtual Protocol protocol() const = 0;
  virtual CoherentActorResult apply(const CoherencyMessage * m) = 0;
  virtual ~CoherentActorBase() {}
};

class CoherentAgentModel : public CoherentActorBase {
 public:
  CoherentAgentModel(const CoherentAgentOptions & opts);
  virtual CoherentActorResult apply(const Transaction * t) = 0;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    Protocol protocol, 
    const CoherentAgentOptions & opts);

struct SnoopFilterOptions {
  SnoopFilterOptions(std::size_t id)
      : id_(id)
  {}

  std::size_t id() const { return id_; }
  
  CacheOptions cache_options;

  std::size_t id_;

  std::size_t num_agents{1};
};

class SnoopFilterModel : public CoherentActorBase {
 public:
  SnoopFilterModel(const SnoopFilterOptions & opts);
};

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    Protocol protocol, 
    const SnoopFilterOptions & opts);

} // namespace ccm

#endif
