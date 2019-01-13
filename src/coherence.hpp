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

#ifndef __SRC_COHERENCE_HPP__
#define __SRC_COHERENCE_HPP__

#include "sim.hpp"
#include "genericcache.hpp"
#include "message.hpp"
#include "actors.hpp"
#include "platform.hpp"
#include <memory>
#include <optional>

namespace ccm {

class Transaction;
class Frontier;

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

const char * to_string(Protocol p);

using state_t = uint8_t;
using result_t = uint8_t;
using command_t = uint8_t;

struct CoherentAgentOptions : ActorOptions {
  CoherentAgentOptions(std::size_t id, Protocol protocol, CacheOptions cache_options, Platform platform)
    : ActorOptions(id), protocol_(protocol), cache_options_(cache_options), platform_(platform)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
  Platform platform() const { return platform_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
  Platform platform_;
};

struct SnoopFilterOptions : ActorOptions {
  SnoopFilterOptions(std::size_t id, Protocol protocol, CacheOptions cache_options, Platform platform)
    : ActorOptions(id), protocol_(protocol), cache_options_(cache_options), platform_(platform)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
  Platform platform() const { return platform_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
  Platform platform_;
};

#define AGENT_COMMANDS(__func)                  \
  __func(UpdateState)                           \
  __func(SetAckCount)                           \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitDataToReq)                         \
  __func(EmitDataToDir)                         \
  __func(EmitInvAck)

enum class CoherentAgentCommand : command_t {
#define __declare_state(__state)                \
  __state,
  AGENT_COMMANDS(__declare_state)
#undef __declare_state
};

const char * to_string(CoherentAgentCommand command);

#define SNOOP_FILTER_COMMANDS(__func)      \
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

enum class SnoopFilterCommand : command_t {
#define __declare_state(__state)                \
  __state,
SNOOP_FILTER_COMMANDS(__declare_state)
#undef __declare_state
};

const char * to_string(SnoopFilterCommand command);

struct CacheLine {
  using state_type = state_t;
  using ack_count_type = uint8_t;

  ack_count_type ack_count() const { return ack_count_; }
  void set_ack_count(ack_count_type ack_count) { ack_count_ = ack_count; }
  
  state_type state() const { return state_; }
  void set_state(state_type state) { state_ = state; }
 private:
  state_type state_;
  ack_count_type ack_count_;
};

struct DirectoryEntry : CacheLine {
  friend std::string to_string(const DirectoryEntry & d);

  using state_type = uint8_t;
  
  DirectoryEntry() {}
    
  //
  std::size_t owner() const { return owner_.value(); }
  void set_owner(std::size_t owner) { owner_ = owner; }
  void clear_owner() { owner_.reset(); }

  //
  const std::vector<std::size_t> & sharers() const { return sharers_; }
  void add_sharer(std::size_t id) { sharers_.push_back(id); }
  void remove_sharer(std::size_t id) {
    sharers_.erase(std::find(sharers_.begin(), sharers_.end(), id), sharers_.end());
  }
  void clear_sharers() { sharers_.clear(); }
  std::size_t num_sharers() const { return sharers_.size(); }

  // TODO: refactor the size of the sharer set, minus the current ID (if
  // present in the sharer list). Denotes the number of invalidations
  // to emit to everything but the requester.
  std::size_t num_sharers_not_id(std::size_t id) const {
    return sharers_.size() - std::count(sharers_.begin(), sharers_.end(), id);
  }
  

 private:
  // Current set of sharing agents
  std::vector<std::size_t> sharers_;

  // Current owner agent
  std::optional<std::size_t> owner_;
};

std::string to_string(const DirectoryEntry & d);

#define TRANSACTION_RESULT(__func)              \
  __func(Hit)                                   \
  __func(Miss)                                  \
  __func(Blocked)

enum TransactionResult : result_t {
#define __declare_state(__state)                \
  __state,
  TRANSACTION_RESULT(__declare_state)
#undef __declare_state
};

const char * to_string(TransactionResult r);

#define MESSAGE_RESULT(__func)                  \
  __func(Commit)                                \
  __func(Stall)

enum MessageResult : result_t {
#define __declare_state(__state)                \
  __state,
  MESSAGE_RESULT(__declare_state)
#undef __declare_state
};

#define ACTION_FIELDS(__func)                   \
  __func(transaction_done, bool)                \
  __func(next_state, state_t)                   \
  __func(is_exclusive, bool)                    \
  __func(error, bool)                           \
  __func(ack_count, uint8_t)                    \
  __func(result, result_t)                      \
  __func(commands, std::vector<command_t>)

struct CoherenceActions {
  CoherenceActions() { reset(); }
#define __declare_getter_setter(__name, __type) \
  using __name ## _type = __type;               \
  __type __name() const { return __name ## _; } \
  template<typename T>                          \
  void set_ ## __name(const T & __name) {       \
    __name ## _ = static_cast<__type>(__name);  \
  }
  ACTION_FIELDS(__declare_getter_setter)
#undef __declare_getter_setter

  // TODO
  Time duration() const { return 10; }
  
  template<typename T>
  void append_command(const T & cmd) {
    commands_.push_back(static_cast<command_t>(cmd));
  }
  
 private:
  void reset() {
    is_exclusive_ = false;
    error_ = false;
    ack_count_ = 0;
    transaction_done_ = false;
  }
#define __declare_field(__name, __type)         \
  __type __name ## _;
  ACTION_FIELDS(__declare_field)
#undef __declare_field
};

class CoherentActorBase {
 public:
  CoherentActorBase() {}
  virtual ~CoherentActorBase() {}
  
  virtual Protocol protocol() const = 0;
 private:
};

class CoherentAgentModel : public CoherentActorBase {
 public:
  CoherentAgentModel(const CoherentAgentOptions & opts);
  virtual ~CoherentAgentModel() {}
  
  virtual void init(CacheLine & l) const = 0;
  virtual bool is_stable(const CacheLine & l) const = 0;
  virtual std::string to_string(state_t l) const = 0;
  
  virtual CoherenceActions get_actions(
      const Message * t, const CacheLine & cache_line) const = 0;
  virtual CoherenceActions get_actions(
      const Transaction * t, const CacheLine & cache_line) const = 0;
};

struct CoherentAgentCommandInvoker : CoherentActor {
  using ack_count_type = std::size_t;
  
  CoherentAgentCommandInvoker(const CoherentAgentOptions & opts);

  Time time() const { return time_; }
  CacheLine cache_line(std::size_t addr) const;
  
  void execute(
      Context & ctxt, const CoherenceActions & actions,
      CacheLine & cache_line, Transaction * t);
  void set_time(std::size_t time) { time_ = time; }

  // protected:
  std::unique_ptr<CoherentAgentModel> cc_model_;
  std::unique_ptr<GenericCache<CacheLine> > cache_;
 private:
  void execute_update_state(
      Context & ctxt, CacheLine & cache_line, state_t state_next);
  void execute_set_ack_count(
      CacheLine & cache_line, ack_count_type ack_count);
  void execute_emit_gets(Context & ctxt, Transaction * t);
  void execute_emit_getm(Context & ctxt, Transaction * t);
  void execute_emit_data_to_req(Context & ctxt, Transaction * t);
  void execute_emit_data_to_dir(Context & ctxt, Transaction * t);
  void execute_emit_inv_ack(Context & ctxt, Transaction * t);
  
  MessageDirector msgd_;
  std::size_t id_;
  Time time_;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts);

class SnoopFilterModel : public CoherentActorBase {
 public:
  SnoopFilterModel(const SnoopFilterOptions & opts);
  
  virtual void init(DirectoryEntry & l) const = 0;
  virtual bool is_stable(const DirectoryEntry & l) const = 0;
  virtual std::string to_string(const DirectoryEntry & l) const = 0;
  virtual std::string to_string(state_t l) const = 0;
  
  virtual CoherenceActions get_actions(
      const Message * t, const DirectoryEntry & dir_entry) const = 0;
 private:
  const SnoopFilterOptions opts_;
};

struct SnoopFilterCommandInvoker : CoherentActor {
  SnoopFilterCommandInvoker(const SnoopFilterOptions & opts);
  
  Time time() const { return time_; }
  DirectoryEntry directory_entry(std::size_t addr) const;
  
  void execute(
      Context & ctxt, const CoherenceActions & actions,
      const Message * msg, DirectoryEntry & d);
  void set_time(Time time) { time_ = time; }
  // protected:
  std::unique_ptr<SnoopFilterModel> cc_model_;
  std::unique_ptr<GenericCache<DirectoryEntry> > cache_;
private:
  void execute_update_state(
      Context & ctxt, DirectoryEntry & d, state_t state_next);
  void execute_set_owner_to_req(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_send_data_to_req(
      const Message * msg, Context & ctxt, DirectoryEntry & d, const CoherenceActions & act);
  void execute_send_inv_to_sharers(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_clear_sharers(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_add_req_to_sharers(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_del_req_from_sharers(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_del_owner(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_add_owner_to_sharers(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_cpy_data_to_memory(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_send_put_sack_to_req(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_send_put_mack_to_req(
      const Message * msg, Context & ctxt, DirectoryEntry & d);
  void execute_send_fwd_gets_to_owner(
      const Message * msg, Context & ctxt, DirectoryEntry & d);

  MessageDirector msgd_;
  std::size_t id_;
  Time time_;
  const SnoopFilterOptions opts_;
};

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    const SnoopFilterOptions & opts);

} // namespace ccm

#endif
