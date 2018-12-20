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
#include <memory>
#include <vector>

namespace ccm {

enum class Protocol {
  MSI
};

#define MESSAGE_CLASSES(__func)                 \
  __func(Load)                                  \
  __func(Store)                                 \
  __func(GetS)                                  \
  __func(GetM)                                  \
  __func(Replacement)                           \
  __func(FwdGetS)                               \
  __func(FwdGetM)                               \
  __func(Inv)                                   \
  __func(PutAck)                                \
  __func(DataDir)                               \
  __func(DataOwner)                             \
  __func(InvAck)

enum class MessageType {
#define __declare_enum(e) e,
  MESSAGE_CLASSES(__declare_enum)
#undef __declare_enum
};
const char * to_string(MessageType t);

struct Addr {
  using type = uint64_t;

  Addr() : d_(0) {}
  Addr(type d) : d_(d) {}

  //
  operator type() { return d_; }

  //
  type line(type len) const { return d_ >> kernel::log2ceil(len); }
  type offset(type len) const { return d_ & ((1 << kernel::log2ceil(len)) - 1); }
  type tag(type len, type ways_n) {
    return line(len) >> kernel::log2ceil(ways_n);
  }
 private:
  type d_;
};

class CoherencyMessage : public kernel::Transaction {
  friend class CoherencyMessageBuilder;
  friend class GetSCoherencyMessageBuilder;
  friend class GetMCoherencyMessageBuilder;
  
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
      : msg_(msg)
  {}
 public:
  GetSCoherencyMessage * msg() { return msg_; }
  
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

class GetMCoherencyMessageBuilder {
  friend class GetMCoherencyMessageDirector;

  GetMCoherencyMessageBuilder(GetMCoherencyMessage * msg)
      : msg_(msg)
  {}
 public:
  GetMCoherencyMessage * msg() { return msg_; }
  
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

class ReplacementCoherencyMessage : public CoherencyMessage {
};

class FwdGetSCoherencyMessage : public CoherencyMessage {
};

class FwdGetMCoherencyMessage : public CoherencyMessage {
};

class InvCoherencyMessage : public CoherencyMessage {
};

class PutAckCoherencyMessage : public CoherencyMessage {
};

class DataDirCoherencyMessage : public CoherencyMessage {
};

class DataOwnerCoherencyMessage : public CoherencyMessage {
};

class InvAckCoherencyMessage : public CoherencyMessage {
};

#define EVICTION_POLICIES(__func)               \
  __func(Fixed)                                 \
  __func(Random)                                \
  __func(PsuedoLru)                             \
  __func(TrueLru)

enum class EvictionPolicy {
#define __declare_eviction_policy(p) p,
  EVICTION_POLICIES(__declare_eviction_policy)
#undef __declare_eviction_policy
};
const char * to_string(EvictionPolicy p);

struct GenericCacheModelOptions {
  bool is_valid() const { return true; }
  
  uint32_t sets_n{1 << 10};
  uint8_t ways_n{1};
  uint8_t line_bytes_n{64};
  EvictionPolicy eviction_policy{EvictionPolicy::Fixed};
};

template<typename T>
class GenericCacheModel {
 public:
  using momento_type = T;
  
  GenericCacheModel(const GenericCacheModelOptions & opts)
      : opts_(opts) {
    ts_.resize(opts_.sets_n * opts_.ways_n);
    invalidate();
  }
  
  //
  bool is_hit(Addr a, momento_type & m) {
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++) {
      if (ts_[i].is_valid && (ts_[i].tag == a.tag(opts_.line_bytes_n, opts_.ways_n))) {
        m = ts_[i].momento;
        return true;
      }
    }
    return false;
  }
  
  bool requires_eviction(Addr a) const {
    uint8_t valid = 0;
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++)
      if (ts_[i].is_valid)
        valid++;
    return (valid == opts_.ways_n);
  }

  //
  void install(Addr a, const momento_type & m) {
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++) {
      if (!ts_[i].is_valid) {
        ts_[i].is_valid = true;
        ts_[i].tag = a.tag(opts_.line_bytes_n, opts_.ways_n);
        ts_[i].momento = m;
      }
    }
  }
  
  void evict(Addr a, momento_type & m) {
    switch (opts_.eviction_policy) {
      case EvictionPolicy::Fixed: {
        CacheEntry & e = ts_[line_set_base(a)];
        m = e.momento;
        e.is_valid = false;
      } break;
      case EvictionPolicy::Random: {
      
      } break;
      case EvictionPolicy::PsuedoLru: {
      
      } break;
      case EvictionPolicy::TrueLru: {
      
      } break;
      default:
        ;
        // TODO
    }
  }

 private:
  std::size_t line_set_base(Addr a) const {
    return opts_.ways_n * a.line(opts_.line_bytes_n);
  }
  
  void invalidate() {
    for (CacheEntry & ce : ts_)
      ce.is_valid = false;
  }    
  
  struct CacheEntry {
    T momento;
    Addr::type tag;
    bool is_valid;
  };
  
  GenericCacheModelOptions opts_;
  std::vector<CacheEntry> ts_;
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
  __func(Miss)                                  \
  __func(NotifyOnRequestReissue)

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
