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

namespace ccm {

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

#define AGENT_MESSAGE_CLASSES(__func)           \
  __func(Load)                                  \
  __func(Store)                                 \
  __func(FwdGetS)                               \
  __func(FwdGetM)

#define DIRECTORY_MESSAGE_CLASSES(__func)       \
  __func(GetS)                                  \
  __func(GetM)                                  \
  __func(PutS)                                  \
  __func(PutM)

enum class MessageType {
#define __declare_enum(e) e,
  AGENT_MESSAGE_CLASSES(__declare_enum)
  DIRECTORY_MESSAGE_CLASSES(__declare_enum)
#undef __declare_enum
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
  
 public:
  MessageType type() const { return type_; }
  std::size_t tid() const { return tid_; }
 private:
  MessageType type_;
  std::size_t tid_;
};

class CoherencyMessageBuilder {
 public:
  void set_tid(std::size_t id) { msg_->tid_ = id; }
 private:
  CoherencyMessage * msg_;
};

class LoadCoherencyMessage : public CoherencyMessage {
  friend class LoadCoherencyMessageBuilder;
 public:
  Addr addr() const { return addr_; }
 private:
  Addr addr_;
};

class StoreCoherencyMessage : public CoherencyMessage {
 public:
  Addr addr() const { return addr_; }
 private:
  Addr addr_;
};

class GetSCoherencyMessage : public CoherencyMessage {
  friend class GetSCoherencyMessageBuilder;
 public:
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class GetSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class GetSCoherencyMessageDirector;

  GetSCoherencyMessageBuilder(GetSCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~GetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  GetSCoherencyMessage * msg() {
    GetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class GetMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class GetMCoherencyMessageDirector;

  GetMCoherencyMessageBuilder(GetMCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~GetMCoherencyMessageBuilder() { if (msg_) msg_->release(); }
  GetMCoherencyMessage * msg() {
    GetMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return m;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class PutSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class PutSCoherencyMessageDirector;

  PutSCoherencyMessageBuilder(PutSCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~PutSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutSCoherencyMessage * msg() {
    PutSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class PutMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class PutMCoherencyMessageDirector;

  PutMCoherencyMessageBuilder(PutMCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~PutMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  PutMCoherencyMessage * msg() {
    PutMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class FwdGetSCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class FwdGetSCoherencyMessageDirector;

  FwdGetSCoherencyMessageBuilder(FwdGetSCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~FwdGetSCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetSCoherencyMessage * msg() {
    FwdGetSCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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
  Addr addr() const { return addr_; }
  void reset() override {}
 private:
  Addr addr_;
};

class FwdGetMCoherencyMessageBuilder : public CoherencyMessageBuilder {
  friend class FwdGetMCoherencyMessageDirector;

  FwdGetMCoherencyMessageBuilder(FwdGetMCoherencyMessage * msg)
      : msg_(msg) {}
 public:
  ~FwdGetMCoherencyMessageBuilder() { if (msg_) { msg_->release(); } }
  FwdGetMCoherencyMessage * msg() {
    FwdGetMCoherencyMessage * m{nullptr};
    std::swap(m, msg_);
    return msg_;
  }
  
  void set_addr(Addr a) { msg_->addr_ = a; }
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

struct CoherentAgentOptions {
  Protocol protocol;
  GenericCacheModelOptions cache_options;
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
  ResponseType resp;
  CoherencyMessage * msg{nullptr};
};

class CoherentAgentModel {
 public:
  CoherentAgentModel(const CoherentAgentOptions & opts);
  
  virtual Protocol protocol() const = 0;
  virtual CoherentAgentAction apply(CoherencyMessage * m) = 0;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts);

struct DirectoryOptions {
  Protocol protocol;
  GenericCacheModelOptions cache_options;
};

struct DirectoryAction {
};

class DirectoryModel {
 public:
  DirectoryModel(const DirectoryOptions & opts);
  
  virtual Protocol protocol() const = 0;
  virtual DirectoryAction apply(CoherencyMessage * m) = 0;
};

std::unique_ptr<DirectoryModel> directory_factory(
    const DirectoryOptions & opts);

} // namespace ccm

#endif
