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

#ifndef __SRC_AGENT_HPP__
#define __SRC_AGENT_HPP__

#include "coherence.hpp"
#include "message.hpp"
#include "sim.hpp"
#include "options.hpp"
#ifdef ENABLE_JSON
#  include <nlohmann/json.hpp>
#  include <memory>
#endif

namespace ccm {

class LoggerScope;
struct TransactionSource;

struct AgentOptions : CoherentAgentOptions {
  AgentOptions(std::size_t id, Platform platform, CacheOptions cache_options)
      : CoherentAgentOptions(id, platform, cache_options) {}
#ifdef ENABLE_JSON
  static AgentOptions from_json(
      const Platform & platform, LoggerScope *l, nlohmann::json j);
#endif
};

struct CoherentAgentCommandInvoker : CoherentActor {
  using ack_count_type = std::size_t;

  CoherentAgentCommandInvoker(const CoherentAgentOptions& opts);

  CacheLine cache_line(std::size_t addr) const;

  void visit_cache(CacheVisitor* cache_visitor) const override;
  void execute(Context& context, Cursor& cursor,
               const CoherenceActions& actions,
               const Transaction * t);
  void execute(Context& context, Cursor& cursor,
               const CoherenceActions& actions,
               const Message* msg);

 protected:
  std::unique_ptr<AgentProtocol> cc_model_;
  std::unique_ptr<GenericCache<CacheLine> > cache_;

 private:
  // Common
  //
  void execute_update_state(const Transaction* t, state_t next_state);
  void execute_set_ack_expect_count(const Message * msg);
  void execute_inc_ack_count(const Transaction * t);
  void execute_emit_gets(Context& context, Cursor& cursor,
                         const Transaction* t);
  void execute_emit_getm(Context& context, Cursor& cursor,
                         const Transaction* t);
  void execute_emit_puts(Context& context, Cursor& cursor,
                         const Transaction* t);
  void execute_emit_pute(Context& context, Cursor& cursor,
                         const Transaction* t);
  void execute_emit_puto(Context& context, Cursor& cursor,
                         const Transaction* t);
  void execute_emit_data_to_dir(Context& context, Cursor& cursor,
                                const Transaction * t);
  void execute_emit_data_to_req(Context& context, Cursor& cursor,
                                const Message* msg);
  void execute_emit_inv_ack(Context& context, Cursor& cursor,
                            const Message* msg);

  MessageDirector msgd_;
};

class Agent : public CoherentAgentCommandInvoker {
  enum class CommandType { Message, Transaction, Invalid };

  struct CommandArbitrator {
    CommandArbitrator(const TransactionQueueManager & tq,
                      MessageQueueManager & mq)
        : tq_(tq), mq_(mq) {}
    ~CommandArbitrator();

    bool is_valid() const { return command_type_ != CommandType::Invalid; }
    Time frontier() const { return frontier_; }
    CommandType command_type() const { return command_type_; }

    void arbitrate();
    void disable_message_class(MessageClass::type cls);
    void disable_transactions() { consider_transactions_ = false; }
   private:
    bool consider_transactions_{true};
    Time frontier_{0};
    CommandType command_type_{CommandType::Invalid};
    const TransactionQueueManager & tq_;
    MessageQueueManager::Proxy mq_;
  };

  struct Statistics {
    std::size_t hits_n{0}, misses_n{0};
  };
  
 public:
  Agent(const AgentOptions& opts);

  bool is_active() const override;
  Protocol::type protocol() const { return opts_.protocol(); }
  CacheOptions cache_options() const { return opts_.cache_options(); }

  void set_transaction_source(std::unique_ptr<TransactionSource> && trns) {
    trns_ = std::move(trns);
  }
  TransactionSource* transaction_source() const { return trns_.get(); }

  void apply(TimeStamped<Message*> ts) override;
  void eval(Context& ctxt) override;

 private:
  void fetch_transactions(std::size_t n = 10);

  result_t handle_message(
      Context & context, Cursor & cursor, CommandArbitrator & arb);
  result_t handle_transaction(
      Context & context, Cursor & cursor, CommandArbitrator & arb);

  CoherenceActions get_actions(Context& ctxt, Cursor& cursor, const Transaction *t);
  CoherenceActions get_actions(Context& ctxt, Cursor& cursor, const Message *msg);
  void enqueue_replacement(const Cursor & cursor, const Transaction * t);

  TransactionQueueManager tq_;
  MessageQueueManager mq_;
  std::unique_ptr<TransactionSource> trns_;
  TransactionFactory tfac_;
  const AgentOptions opts_;
  Statistics stats_;
};
#ifdef ENABLE_JSON

struct AgentBuilder {
  static std::unique_ptr<Agent> construct(
      const Platform & platform, LoggerScope * l, nlohmann::json j);
};
#endif

}  // namespace ccm

#endif
