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

#include "agent.hpp"
#include <queue>
#include "common.hpp"

namespace ccm {

const char* CoherentAgentCommand::to_string(command_t command) {
  switch (command) {
    // clang-format off
#define __declare_to_string(__e)                \
  case CoherentAgentCommand::__e:           \
    return #__e;
  AGENT_COMMANDS(__declare_to_string)
#undef __declare_to_string
      // clang-format on
    default:
      return "<Invalid Line State>";
  }
}

class AgentMessageAdmissionControl : public MessageAdmissionControl {
 public:
  AgentMessageAdmissionControl(Agent* agent) : agent_(agent) {}

  bool can_be_issued(const Message* msg) const override {
    const Transaction* trn = msg->transaction();
    CacheLine& cache_line = agent_->cache_->lookup(trn->addr());
    const CoherenceActions actions =
        agent_->cc_model_->get_actions(msg, cache_line);

    switch (actions.result()) {
      case MessageResult::Stall:
        return false;
        break;

      case MessageResult::Commit:
        return true;
        break;

      default:
        // TODO: Error
        break;
    }

    return true;
  }

 private:
  Agent* agent_{nullptr};
};

class AgentTransactionAdmissionControl : public TransactionAdmissionControl {
 public:
  AgentTransactionAdmissionControl(Agent* agent) : agent_(agent) {}

  bool can_be_issued(const Transaction* trn) const override {
    // TODO: variety of agent related issue conditions (max in flight count,
    // credits, etc..)

    // The coherence protocol cannot block if (by definition) the
    // transaction address is not present in the agents cache.
    //
    if (!agent_->cache_->is_hit(trn->addr())) return true;

    // If the address is present in the agents cache, explicitly
    // derive the set of actions to be applied and verify that the
    // protocol may advance.
    //
    // This is an expensive operation.
    //
    CacheLine& cache_line = agent_->cache_->lookup(trn->addr());
    const CoherenceActions actions =
        agent_->cc_model_->get_actions(trn, cache_line);
    switch (actions.result()) {
      case TransactionResult::Blocked:
        return false;
        break;

      case TransactionResult::Hit:
      case TransactionResult::Miss:
        return true;
        break;

      default:
        // Error
        return false;
    }
  }

 private:
  Agent* agent_{nullptr};
};

CoherentAgentCommandInvoker::CoherentAgentCommandInvoker(
    const CoherentAgentOptions& opts)
    : CoherentActor(opts), msgd_(opts) {
  cc_model_ = coherent_agent_factory(opts.protocol(), opts);
  cache_ = cache_factory<CacheLine>(opts.cache_options());
}

CacheLine CoherentAgentCommandInvoker::cache_line(std::size_t addr) const {
  CacheLine cl;
  if (cache_->is_hit(addr)) {
    cl = cache_->lookup(addr);
  } else {
    cc_model_->init(cl);
  }
  return cl;
}

void CoherentAgentCommandInvoker::visit_cache(
    CacheVisitor* cache_visitor) const {
  cache_visitor->set_id(id());
  cache_->visit(cache_visitor);
}

void CoherentAgentCommandInvoker::execute(Context& context, Cursor& cursor,
                                          const CoherenceActions& actions,
                                          CacheLine& cache_line,
                                          const Transaction* t) {
  for (command_t cmd : actions.commands()) {
    log_debug("Execute: ", CoherentAgentCommand::to_string(cmd));

    switch (cmd) {
      case CoherentAgentCommand::UpdateState:
        execute_update_state(cache_line, actions);
        break;
      case CoherentAgentCommand::EmitGetS:
        execute_emit_gets(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitGetM:
        execute_emit_getm(context, cursor, t);
        break;
      default:
        break;
    }
  }
}

void CoherentAgentCommandInvoker::execute(Context& context, Cursor& cursor,
                                          const CoherenceActions& actions,
                                          CacheLine& cache_line,
                                          const Message* msg) {
  for (command_t cmd : actions.commands()) {
    log_debug("Execute: ", CoherentAgentCommand::to_string(cmd));

    switch (cmd) {
      case CoherentAgentCommand::UpdateState:
        execute_update_state(cache_line, actions);
        break;
      case CoherentAgentCommand::EmitDataToReq:
        execute_emit_data_to_req(context, cursor, msg);
        break;
      case CoherentAgentCommand::EmitDataToDir:
        execute_emit_data_to_dir(context, cursor, msg);
        break;
      case CoherentAgentCommand::EmitInvAck:
        execute_emit_inv_ack(context, cursor, msg);
        break;
      case CoherentAgentCommand::SetAckCount:
        execute_set_ack_count(cache_line, actions);
        break;
      default:
        break;
    }
  }
}

void CoherentAgentCommandInvoker::execute_update_state(
    CacheLine& cache_line, const CoherenceActions& actions) {
  const state_t next_state = actions.next_state();
  log_debug("Update state; current: ", cc_model_->to_string(next_state),
            " previous: ", cc_model_->to_string(cache_line.state()));
  cache_line.set_state(next_state);
}

void CoherentAgentCommandInvoker::execute_emit_gets(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::GetS);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  log_debug("Sending GetS to home directory.");
  emit_message(context, cursor, b);
}

void CoherentAgentCommandInvoker::execute_emit_getm(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::GetM);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  log_debug("Sending GetM to home directory.");
  emit_message(context, cursor, b);
}

void CoherentAgentCommandInvoker::execute_emit_data_to_req(Context& context,
                                                           Cursor& cursor,
                                                           const Message* msg) {
  const Transaction* t = msg->transaction();
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::Data);
  b.set_dst_id(msg->fwd_id());
  b.set_transaction(t);

  log_debug("Emit Data To Requester.");
  emit_message(context, cursor, b);
}

void CoherentAgentCommandInvoker::execute_emit_data_to_dir(Context& context,
                                                           Cursor& cursor,
                                                           const Message* msg) {
  const Transaction* t = msg->transaction();
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::Data);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  log_debug("Emit Data To Directory.");
  emit_message(context, cursor, b);
}

void CoherentAgentCommandInvoker::execute_emit_inv_ack(Context& context,
                                                       Cursor& cursor,
                                                       const Message* msg) {
  const Transaction* t = msg->transaction();
  MessageBuilder b = msgd_.builder();

  b.set_type(MessageType::Inv);
  b.set_is_ack(true);

  // TODO: This is a special case because the dst_id for the ack. is
  // not the originator of the command.
  b.set_dst_id(msg->src_id());
  b.set_dst_id(0);
  b.set_transaction(t);

  log_debug("Sending invalidation acknowledgement.");
  emit_message(context, cursor, b);
}

void CoherentAgentCommandInvoker::execute_set_ack_count(
    CacheLine& cache_line, const CoherenceActions& actions) {
  const CoherenceActions::ack_count_type ack_count = actions.ack_count();
  log_debug("Update ack_count: ", (int)ack_count);
  cache_line.set_ack_count(ack_count);
}

Agent::Agent(const AgentOptions& opts)
    : CoherentAgentCommandInvoker(opts), opts_(opts) {
  set_time(0);
  set_logger_scope(opts.logger_scope());

  qmgr_.set_ac(std::make_unique<AgentMessageAdmissionControl>(this));
  qmgr_.set_ac(std::make_unique<AgentTransactionAdmissionControl>(this));
}

void Agent::eval(Context& context) {
  const Epoch epoch = context.epoch();
  Cursor cursor = epoch.cursor();

  // The agent is present at 'time()'; execute Agent actions iff the
  // time is present in the current Epoch, otherwise, sleep until the
  // next interval.
  //
  if (!epoch.in_interval(time())) return;

  // As the current time may not fall on a Epoch boundary, advance to
  // the 'time' location within the current Epoch and proceed from
  // there.
  //
  cursor.set_time(std::max(time(), cursor.time()));

  if (!qmgr_.pending_transactions()) fetch_transactions(10);

  do {
    const QueueEntry next = qmgr_.next();
    if (next.type() == QueueEntryType::Invalid) break;

    if (!epoch.in_interval(next.time())) break;

    cursor.set_time(std::max(time(), next.time()));
    set_time(cursor.time());

    CoherenceActions actions;
    switch (next.type()) {
      case QueueEntryType::Message:
        handle_msg(context, cursor, next.as_msg());
        break;

      case QueueEntryType::Transaction:
        handle_trn(context, cursor, next.as_trn());
        break;

      default:
        break;
    }
    next.consume();
  } while (epoch.in_interval(cursor.time()));

  set_time(cursor.time());
}

void Agent::fetch_transactions(std::size_t n) {
  TimeStamped<Transaction*> ts;
  for (int i = 0; i < n; i++) {
    if (!trns_->get_transaction(ts)) break;

    qmgr_.push(ts);
  }
}

void Agent::handle_msg(Context& context, Cursor& cursor,
                       TimeStamped<Message*> ts) {
  const Message* msg = ts.t();
  const Transaction* trn = msg->transaction();

  CCM_AGENT_ASSERT(cache_->is_hit(trn->addr()));
  CacheLine& cache_line = cache_->lookup(trn->addr());
  const CoherenceActions actions = cc_model_->get_actions(msg, cache_line);
  CCM_AGENT_ASSERT(!actions.error());

  execute(context, cursor, actions, cache_line, msg);
  // TODO: assertion that confirms commital

  if (actions.transaction_done())
    trns_->event(TransactionEvent::End,
                 TimeStamped{time(), msg->transaction()});

  msg->release();
}

void Agent::handle_trn(Context& context, Cursor& cursor,
                       TimeStamped<Transaction*> ts) {
  const Transaction* trn = ts.t();

  trns_->event(TransactionEvent::Start, TimeStamped{time(), trn});

  if (trn->type() != TransactionType::Replacement)
    CCM_AGENT_ASSERT(!cache_->requires_eviction(trn->addr()));

  if (!cache_->is_hit(trn->addr())) {
    CacheLine cache_line;
    cc_model_->init(cache_line);
    cache_->install(trn->addr(), cache_line);
  }

  CacheLine& cache_line = cache_->lookup(trn->addr());
  const CoherenceActions actions = cc_model_->get_actions(trn, cache_line);
  CCM_AGENT_ASSERT(!actions.error());

  execute(context, cursor, actions, cache_line, trn);
}

void Agent::apply(TimeStamped<Message*> ts) { qmgr_.push(ts); }

bool Agent::is_active() const { return !qmgr_.empty(); }

}  // namespace ccm
