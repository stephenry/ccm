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

#include "genericcache.hpp"
#include "message.hpp"
#include "actors.hpp"
#include <memory>
#include <optional>

namespace ccm {

class Message;
class Transaction;
class AgentOptions;
class SnoopFilterOptions;
class Frontier;
class CoherentActorActions;

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

struct CoherentAgentOptions : ActorOptions {
  CoherentAgentOptions(std::size_t id, Protocol protocol, CacheOptions cache_options)
      : ActorOptions(id), protocol_(protocol), cache_options_(cache_options)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
};

struct SnoopFilterOptions : ActorOptions {
  SnoopFilterOptions(std::size_t id, Protocol protocol, CacheOptions cache_options)
      : ActorOptions(id), protocol_(protocol), cache_options_(cache_options)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
};

#define AGENT_COMMANDS(__func)                  \
  __func(UpdateState)                           \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitDataToReq)                         \
  __func(EmitDataToDir)                         \
  __func(EmitInvAck)

enum class CoherentAgentCommand : uint8_t {
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
  __func(SendFwdGetSToOwner)

enum class SnoopFilterCommand : uint8_t {
#define __declare_state(__state)                \
  __state,
SNOOP_FILTER_COMMANDS(__declare_state)
#undef __declare_state
};

const char * to_string(SnoopFilterCommand command);

#define COHERENT_ACTOR_RESULT(__func)           \
  __func(Advances)                              \
  __func(BlockedOnProtocol)                     \
  __func(TagsExhausted)

struct CacheLine {
  using state_type = uint8_t;
  
  state_type state() const { return state_; }
  void set_state(state_type state) { state_ = state; }
 private:
  state_type state_;
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

 private:
  // Current set of sharing agents
  std::vector<std::size_t> sharers_;

  // Current owner agent
  std::optional<std::size_t> owner_;
};

std::string to_string(const DirectoryEntry & d);

struct CoherentActorActions {
  using state_type = CacheLine::state_type;
  using action_type = uint8_t;

  bool has_state_update() const { return next_state_.has_value(); }
  state_type next_state() const { return next_state_.value(); }

  template<typename T>
  void set_next_state(T state) { next_state_ = static_cast<state_type>(state); }

  //
  bool error() const { return false; }
  void set_error() {}

  //
  bool stall() const { return stall_; }
  void set_stall() { stall_ = true; }

  std::size_t message_count() const { return msgs_n_; }

  const std::vector<action_type> & actions() const { return actions_; }

  template<typename T>
  void add_action(T a) {
    actions_.push_back(static_cast<action_type>(a));
  }

 private:
  bool stall_{false};
  std::optional<state_type> next_state_;
  std::vector<action_type> actions_;
  std::size_t msgs_n_{0};
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
  
  virtual void line_init(CacheLine & l) const = 0;
  virtual bool line_is_stable(const CacheLine & l) const = 0;
  virtual std::string to_string(CacheLine::state_type l) const = 0;
  
  virtual CoherentActorActions get_actions(const Message * t) const = 0;
  virtual CoherentActorActions get_actions(const Transaction * t) const = 0;
};

struct CoherentAgentCommandInvoker : CoherentActor {
  using state_type = CoherentActorActions::state_type;
  using action_type = CoherentActorActions::action_type;
  
  CoherentAgentCommandInvoker(const CoherentAgentOptions & opts);

  std::size_t time() const { return time_; }
  
  void execute(
      Frontier & f, const CoherentActorActions & actions, CacheLine & cache_line);
  void set_time(std::size_t time) { time_ = time; }

 protected:
  std::unique_ptr<CoherentAgentModel> cc_model_;
  std::unique_ptr<GenericCache<CacheLine> > cache_;
 private:
  void execute_update_state(
      Frontier & f, CacheLine & cache_line, state_type state_next);
  void execute_emit_gets(Frontier & f);
  void execute_emit_getm(Frontier & f);
  void execute_emit_data_to_req(Frontier & f);
  void execute_emit_data_to_dir(Frontier & f);
  void execute_emit_inv_ack(Frontier & f);
  
  MessageDirector msgd_;
  std::size_t id_;
  std::size_t time_;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const CoherentAgentOptions & opts);

class SnoopFilterModel : public CoherentActorBase {
 public:
  SnoopFilterModel(const SnoopFilterOptions & opts);
  
  virtual void init(DirectoryEntry & l) const = 0;
  virtual bool is_stable(const DirectoryEntry & l) const = 0;
  virtual std::string to_string(const DirectoryEntry & l) const = 0;
  virtual std::string to_string(CacheLine::state_type l) const = 0;
  
  virtual CoherentActorActions get_actions(
      const Message * t, const DirectoryEntry & dir_entry) const = 0;
 private:
  const SnoopFilterOptions opts_;
};

struct SnoopFilterCommandInvoker : CoherentActor {
  using state_type = CoherentActorActions::state_type;
  
  SnoopFilterCommandInvoker(const SnoopFilterOptions & opts);
  
  std::size_t time() const { return time_; }

  void execute(
      Frontier & f, const CoherentActorActions & actions,
      const Message * msg, DirectoryEntry & d);
  void set_time(std::size_t time) { time_ = time; }
 protected:
  std::unique_ptr<SnoopFilterModel> cc_model_;
  std::unique_ptr<GenericCache<CacheLine> > cache_;
private:
  void execute_update_state(
      Frontier & f, DirectoryEntry & d, state_type state_next);
  void execute_set_owner_to_req(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_send_data_to_req(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_send_inv_to_sharers(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_clear_sharers(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_add_req_to_sharers(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_del_req_from_sharers(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_del_owner(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_add_owner_to_sharers(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_cpy_data_to_memory(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_send_put_sack_to_req(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_send_put_mack_to_req(
      const Message * msg, Frontier & f, DirectoryEntry & d);
  void execute_send_fwd_gets_to_owner(
      const Message * msg, Frontier & f, DirectoryEntry & d);

  MessageDirector msgd_;
  std::size_t id_;
  std::size_t time_;
  const SnoopFilterOptions opts_;
};

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    const SnoopFilterOptions & opts);

} // namespace ccm

#endif
