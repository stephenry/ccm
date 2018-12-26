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

#include "cache_model.hpp"
#include "message.hpp"
#include <memory>
#include <optional>

namespace ccm {

class Message;
class Transaction;
class AgentOptions;
class Frontier;
class CoherentActorActions;

enum class Protocol {
  MSI,
  MESI,
  MOSI
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

struct CoherentAgentCommandInvoker {
  CoherentAgentCommandInvoker(const AgentOptions & opts);

  std::size_t time() const { return time_; }
  
  void invoke(Frontier & f, const CoherentActorActions & actions);
  void set_time(std::size_t time) { time_ = time; }
 private:
  void invoke_update_state(Frontier & f);
  void invoke_emit_gets(Frontier & f);
  void invoke_emit_getm(Frontier & f);
  void invoke_emit_data_to_req(Frontier & f);
  void invoke_emit_data_to_dir(Frontier & f);
  void invoke_emit_inv_ack(Frontier & f);
  
  MessageDirector msgd_;
  std::size_t id_;
  std::size_t time_;
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

struct CoherentActorActions {

  bool has_state_update() const { return next_state_.has_value(); }
  uint8_t next_state() const { return next_state_.value(); }

  template<typename T>
  void set_next_state(T state) { next_state_ = static_cast<uint8_t>(state); }

  //
  bool error() const { return false; }
  void set_error() {}

  //
  bool stall() const { return stall_; }
  void set_stall() { stall_ = true; }

  std::size_t message_count() const { return msgs_n_; }

  const std::vector<uint8_t> & actions() const { return actions_; }

  template<typename T>
  void add_action(T a) {
    actions_.push_back(static_cast<uint8_t>(a));
  }

 private:
  bool stall_{false};
  std::optional<uint8_t> next_state_;
  std::vector<uint8_t> actions_;
  std::size_t msgs_n_{0};
};

class CoherentActorBase {
 public:
  CoherentActorBase() {}
  virtual ~CoherentActorBase() {}
  
  virtual Protocol protocol() const = 0;
  virtual CoherentActorActions get_actions(const Message * m) = 0;
};

class CoherentAgentModel : public CoherentActorBase {
 public:
  CoherentAgentModel(const AgentOptions & opts);
  virtual ~CoherentAgentModel() {}
  
  virtual CoherentActorActions get_actions(const Message * t) = 0;
  virtual CoherentActorActions get_actions(const Transaction * t) = 0;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    const AgentOptions & opts);

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
