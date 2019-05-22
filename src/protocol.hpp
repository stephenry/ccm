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

#ifndef __SRC_PROTOCOL_HPP__
#define __SRC_PROTOCOL_HPP__

#include <map>
#include <memory>
#include <vector>
#include <optional>
#include "types.hpp"
#include "options.hpp"

namespace ccm {

class Message;
class Transaction;
class Platform;

using result_t = uint8_t;
using state_t = uint8_t;
using command_t = uint8_t;

enum class Protocol { MSI,
#ifdef ENABLE_MESI
                      MESI,
#endif
#ifdef ENABLE_MOSI
                      MOSI,
#endif
                      INVALID };
const char* to_string(Protocol p);

// clang-format off
#define AGENT_COMMANDS(__func)                  \
  __func(UpdateState)                           \
  __func(IncAckCount)                           \
  __func(SetAckExpectCount)                     \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitPutS)                              \
  __func(EmitPutM)                              \
  __func(EmitPutE)                              \
  __func(EmitPutO)                              \
  __func(EmitDataToReq)                         \
  __func(EmitDataToDir)                         \
  __func(EmitInvAck)
// clang-format on

struct CoherentAgentCommand {
  using base_type = command_t;
  
  // clang-format off
  enum : base_type {
#define __declare_state(__state)                \
  __state,
  AGENT_COMMANDS(__declare_state)
#undef __declare_state
  // clang-format on
  };

  static const char* to_string(command_t command);
  static std::size_t to_cost(base_type b);
};


// clang-format off
#define SNOOP_FILTER_COMMANDS(__func)           \
  __func(UpdateState)                           \
  __func(SetOwnerToReq)                         \
  __func(SendDataToReq)                         \
  __func(SendInvToSharers)                      \
  __func(ClearSharers)                          \
  __func(AddReqToSharers)                       \
  __func(DelReqFromSharers)                     \
  __func(DelOwner)                              \
  __func(AddOwnerToSharers)                     \
  __func(CpyDataToMemory)                       \
  __func(SendPutSAckToReq)                      \
  __func(SendPutMAckToReq)                      \
  __func(SendPutEAckToReq)                      \
  __func(SendPutOAckToReq)                      \
  __func(SendAckCountToReq)                     \
  __func(SendFwdGetMToOwner)                    \
  __func(SendFwdGetSToOwner)
// clang-format on

struct SnoopFilterCommand {
  enum : command_t {
    // clang-format off
#define __declare_state(__state) __state,
    SNOOP_FILTER_COMMANDS(__declare_state)
#undef __declare_state
    // clang-format on
  };

  static const char* to_string(command_t command);
};

// clang-format off
#define TRANSACTION_RESULT(__func)              \
  __func(Hit)                                   \
  __func(Consumed)                              \
  __func(Miss)                                  \
  __func(Blocked)
// clang-format on

enum TransactionResult : result_t {
// clang-format off
#define __declare_state(__state)                \
  __state,
  TRANSACTION_RESULT(__declare_state)
#undef __declare_state
  // clang-format on
};

const char* to_string(TransactionResult r);

// clang-format off
#define MESSAGE_RESULT(__func)                  \
  __func(Commit)                                \
  __func(Stall)
// clang-format on

enum MessageResult : result_t {
// clang-format off
#define __declare_state(__state)                \
  __state,
  MESSAGE_RESULT(__declare_state)
#undef __declare_state
  // clang-format on
};

// clang-format off
#define ACTION_FIELDS(__func)                   \
  __func(fwd_id, id_t, 0)                       \
  __func(transaction_done, bool, false)         \
  __func(next_state, state_t, 0)                \
  __func(is_exclusive, bool, false)             \
  __func(error, bool, false)                    \
  __func(ack_expect_valid, bool, false)         \
  __func(ack_expect, uint8_t, 0)                \
  __func(ack_count, uint8_t, 0)                 \
  __func(result, result_t, 0)                   \
  __func(requires_eviction, bool, false)        \
  __func(cost, std::size_t, 0)                  \
  __func(commands, std::vector<command_t>, {})
// clang-format on

struct CoherenceActions {
  CoherenceActions() { reset(); }
// clang-format off
#define __declare_getter_setter(__name, __type, __default)      \
  using __name ## _type = __type;                               \
  __type __name() const { return __name ## _; }                 \
  template<typename T>                                          \
  void set_ ## __name(const T & __name) {                       \
    __name ## _ = static_cast<__type>(__name);                  \
  }
  ACTION_FIELDS(__declare_getter_setter)
#undef __declare_getter_setter
  // clang-format on

  template <typename T>
  void append_command(const T& cmd) { // TODO: remove template
    commands_.push_back(static_cast<command_t>(cmd));
  }

  static std::size_t compute_cost(const CoherenceActions & actions);

 private:
  void reset() {
// clang-format off
#define __declare_default(__name, __type, __default)    \
    __name ## _ = __default;
    ACTION_FIELDS(__declare_default)
#undef __declare_default
    // clang-format on
  }
// clang-format off
#define __declare_field(__name, __type, __default)      \
  __type __name ## _;
  ACTION_FIELDS(__declare_field)
#undef __declare_field
  // clang-format on
};

class ProtocolBase {
 public:
  ProtocolBase() {}
  virtual ~ProtocolBase() {}

  virtual Protocol protocol() const = 0;
};

class CacheLine {
 public:
  using state_type = state_t;
  using ack_count_type = uint8_t;

  CacheLine() { reset(); }

#define CACHE_LINE_FIELDS(__func)               \
  __func(state, state_type, 0)                  \
  __func(inv_ack_expect, std::size_t, 0)        \
  __func(inv_ack_expect_valid, bool, false)     \
  __func(inv_ack_count, std::size_t, 0)
  
#define __declare_get_setter(__name, __type, __default)     \
  void set_##__name(__type __name) { __name##_ = __name; }  \
  __type __name() const { return __name##_; }
  CACHE_LINE_FIELDS(__declare_get_setter)
#undef __declare_getter

  void reset() { set_invalid(); }

  bool is_last_inv_ack() const;
 private:
  void set_invalid();
#define __declare_field(__name, __type, __default) __type __name##_;
  CACHE_LINE_FIELDS(__declare_field)
#undef __declare_field
};

class AgentProtocol : public ProtocolBase {
 public:
  AgentProtocol();
  virtual ~AgentProtocol() {}

  virtual void init(CacheLine& l) const = 0;
  virtual bool is_stable(const CacheLine& l) const = 0;
  virtual std::string to_string(state_t l) const = 0;

  virtual CoherenceActions get_actions(const Message* t,
                                       const CacheLine& cache_line) const = 0;
  virtual CoherenceActions get_actions(const Transaction* t,
                                       const CacheLine& cache_line) const = 0;
};

std::unique_ptr<AgentProtocol> agent_protocol_factory(
    Protocol protocol, const Platform & platform);

class DirectoryEntry {
  friend std::string to_string(const DirectoryEntry& d);

 public:
  using state_type = state_t;

  DirectoryEntry() {}

  state_type state() const;
  const std::vector<id_t>& sharers() const;
  std::size_t num_sharers() const;
  void set_state(state_type state);
  void set_owner(id_t owner);
  id_t owner() const;
  void clear_owner();
  void add_sharer(id_t id);
  void remove_sharer(id_t id);
  void clear_sharers();
  id_t num_sharers_not_id(id_t id) const;

 private:
  state_type state_;
  std::vector<id_t> sharers_;
  std::optional<id_t> owner_; // TODO: consider refactoring this away.
};

std::string to_string(const DirectoryEntry& d);

class SnoopFilterProtocol : public ProtocolBase {
 public:
  SnoopFilterProtocol() {}

  virtual void init(DirectoryEntry& l) const = 0;
  virtual bool is_stable(const DirectoryEntry& l) const = 0;
  virtual std::string to_string(const DirectoryEntry& l) const = 0;
  virtual std::string to_string(state_t l) const = 0;

  virtual CoherenceActions get_actions(
      const Message* t, const DirectoryEntry& dir_entry) const = 0;
};

std::unique_ptr<SnoopFilterProtocol> snoop_filter_protocol_factory(
    Protocol protocol);

struct CacheVisitor {
  virtual ~CacheVisitor() {}

  virtual void set_id(id_t id) {}
  virtual void add_line(addr_t addr, const CacheLine& cache_line) {}
  virtual void add_line(addr_t addr, const DirectoryEntry& directory_entry) {}
};

template <typename T>
using Entry = std::tuple<id_t, const T>;

class CoherenceProtocolValidator {
  struct ProtocolValidatorVisitor;

 public:
  CoherenceProtocolValidator();
  virtual ~CoherenceProtocolValidator() {}

  std::unique_ptr<CacheVisitor> get_cache_visitor();

  bool validate() const;
  virtual bool validate_addr(addr_t addr,
                             const std::vector<Entry<CacheLine> >& lines,
                             const DirectoryEntry& entry) const = 0;

 protected:
  void add_cache_line(id_t id, addr_t addr, const CacheLine& cache_line);
  void add_dir_line(id_t id, addr_t addr,
                    const DirectoryEntry& directory_entry);

  void error(const char* str) const {}

  std::map<addr_t, std::vector<Entry<CacheLine> > > cache_lines_;
  std::map<addr_t, DirectoryEntry> directory_lines_;
};

std::unique_ptr<CoherenceProtocolValidator>
coherence_protocol_validator_factory(Protocol protocol);

}  // namespace ccm

#endif
