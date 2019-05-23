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
#include "protocol.hpp"
#include "log.hpp"

namespace ccm {
#ifdef ENABLE_JSON

AgentOptions AgentOptions::from_json(
    const Platform & platform, LoggerScope * l, nlohmann::json j) {
  const id_t id = j["id"];
  AgentOptions opts{id, platform, CacheOptions::from_json(j["cache_options"])};
  opts.set_logger_scope(l->child_scope(j["name"]));
  return opts;
}
#endif

class AgentMessageAdmissionControl : public MessageAdmissionControl {
 public:
  AgentMessageAdmissionControl(Agent* agent) : agent_(agent) {}

  bool can_be_issued(const Message* msg) const override {
    const Transaction* trn = msg->transaction();
    CacheLine& cache_line = agent_->cache_->lookup(trn->addr());
    const CoherenceActions actions =
        agent_->cc_model_->get_actions(msg, cache_line);

    // TODO: The simulator computes the set of permissible messages to
    // issue to the agent without regard of the time of their
    // arrival. Consequently, there may be some minor reordering,
    // which is entirely consistent with a real system. Unfortunately,
    // the protocol as defined in the spec. do not account for such
    // reordering and instead mark such state transitions are
    // impermissible. To account for this, whenever a transition is
    // marked in error, the message is disallowed in this round.
    //
    // An Agent in the IS_D state awaits Data from the snoop filter
    // but instead receives a FwdGet{S,M} from the snoop filter
    // (placed into a separate message queue) because another agent
    // has simultaneously requested the line in the S- or M- states.
    // The IS_D <- FwdGet{S,M} transition is not defined in the
    // blocking MESI protocol.
    //
    if (actions.error())
      return false;

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
      case TransactionResult::Consumed:
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
  cc_model_ = agent_protocol_factory(opts.protocol(), opts.platform());
  cache_ = cache_factory<CacheLine>(opts.cache_options());
}

CacheLine CoherentAgentCommandInvoker::cache_line(std::size_t addr) const {
  // TODO: deprecate
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

void CoherentAgentCommandInvoker::execute(Context& context,
                                          Cursor & cursor,
                                          const CoherenceActions & actions,
                                          const Transaction * t) {
  for (command_t cmd : actions.commands()) {
    log_debug("Execute: ", CoherentAgentCommand::to_string(cmd));

    switch (cmd) {
      case CoherentAgentCommand::UpdateState:
        execute_update_state(t, actions.next_state());
        break;
      case CoherentAgentCommand::EmitGetS:
        execute_emit_gets(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitGetM:
        execute_emit_getm(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitPutS:
        execute_emit_puts(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitPutE:
        execute_emit_pute(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitPutO:
        execute_emit_puto(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitDataToDir:
        execute_emit_data_to_dir(context, cursor, t);
        break;
      default:
        break;
    }
    cursor.advance(CoherentAgentCommand::to_cost(cmd));
  }
}

void CoherentAgentCommandInvoker::execute(Context& context,
                                          Cursor& cursor,
                                          const CoherenceActions& actions,
                                          const Message* msg) {
  for (command_t cmd : actions.commands()) {
    log_debug("Execute: ", CoherentAgentCommand::to_string(cmd));

    const Transaction * t = msg->transaction();
    switch (cmd) {
      case CoherentAgentCommand::EmitDataToReq:
        execute_emit_data_to_req(context, cursor, msg);
        break;
      case CoherentAgentCommand::EmitDataToDir:
        execute_emit_data_to_dir(context, cursor, t);
        break;
      case CoherentAgentCommand::EmitInvAck:
        execute_emit_inv_ack(context, cursor, msg);
        break;
      case CoherentAgentCommand::UpdateState:
        execute_update_state(t, actions.next_state());
        break;
      case CoherentAgentCommand::IncAckCount:
        execute_inc_ack_count(t);
        break;
      case CoherentAgentCommand::SetAckExpectCount:
        execute_set_ack_expect_count(msg);
        break;
      default:
        break;
    }
    cursor.advance(CoherentAgentCommand::to_cost(cmd));
  }
}

void CoherentAgentCommandInvoker::execute_update_state(const Transaction* t,
                                                       state_t next_state) {
  CacheLine & cache_line = cache_->lookup(t->addr());
  
  log_debug("Update state; current: ", cc_model_->to_string(next_state),
            " previous: ", cc_model_->to_string(cache_line.state()));
  cache_line.set_state(next_state);
}

void CoherentAgentCommandInvoker::execute_emit_gets(Context& context,
                                                    const Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::GetS);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Sending GetS to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_getm(Context& context,
                                                    const Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::GetM);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Sending GetM to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_puts(Context& context,
                                                    const Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutS);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Sending PutS to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_pute(Context& context,
                                                    const Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutE);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Sending PutE to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_puto(Context& context,
                                                    const Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutO);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Sending PutO to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_data_to_dir(Context& context,
                                                           const Cursor& cursor,
                                                           const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::Data);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug("Emit Data To Directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_data_to_req(Context& context,
                                                           const Cursor& cursor,
                                                           const Message* msg) {
  const Transaction* t = msg->transaction();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::Data);
  b.set_dst_id(msg->fwd_id());
  // TODO: is this really necessary; should come from the directory.
  b.set_ack_count(msg->ack_count());
  b.set_transaction(t);

  Message *out = b.msg();
  log_debug("Emit Data To Requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});
}

void CoherentAgentCommandInvoker::execute_emit_inv_ack(Context& context,
                                                       const Cursor& cursor,
                                                       const Message* msg) {
  MessageBuilder b = msgd_.builder();
  b.set_src_id(id());
  b.set_type(MessageType::Inv);
  b.set_is_ack(true);
  b.set_dst_id(msg->fwd_id());
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug("Sending invalidation acknowledgement.", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});
}

void CoherentAgentCommandInvoker::execute_inc_ack_count(const Transaction * t) {
  CacheLine & cache_line = cache_->lookup(t->addr());
  cache_line.set_inv_ack_count(cache_line.inv_ack_count() + 1);
  log_debug("Update invalidation count: ", cache_line.inv_ack_count());
}

void CoherentAgentCommandInvoker::execute_set_ack_expect_count(const Message * msg) {
  const Transaction * t = msg->transaction();
  CacheLine & cache_line = cache_->lookup(t->addr());
  cache_line.set_inv_ack_expect_valid(true);
  cache_line.set_inv_ack_expect(msg->ack_count());
  log_debug("Update expected invalidation count: ", cache_line.inv_ack_expect());
}

Agent::CommandArbitrator::~CommandArbitrator() {
  mq_.clr_disregard_class();
}

void Agent::CommandArbitrator::disable_message_class(MessageClass::type cls) {
  mq_.add_disregard_class(cls);
}

void Agent::CommandArbitrator::arbitrate() {
  MinHeap<std::pair<CommandType, Time> > mh;
  if (mq_.is_active()) {
    mq_.recompute_front();
    const MessageQueueManager::TSMessage tsm{mq_.front()};
    mh.push(std::make_pair(CommandType::Message, tsm.time()));
  }
  if (consider_transactions_ && tq_.is_active()) {
    const TransactionQueueManager::TSTransaction tst{tq_.front()};
    mh.push(std::make_pair(CommandType::Transaction, tst.time()));
  }
  if (mh.empty()) { command_type_ = CommandType::Invalid; return; }

  const std::pair<CommandType, Time> top = mh.top();
  command_type_ = top.first;
  frontier_ = top.second;
}

Agent::Agent(const AgentOptions& opts)
    : CoherentAgentCommandInvoker(opts), opts_(opts) {
  set_time(0);
  set_logger_scope(opts.logger_scope());
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

  if (!tq_.is_active()) fetch_transactions(10);


  bool halt{false};
  CommandArbitrator arb{tq_, mq_};
  do {
    arb.arbitrate();
    switch (arb.command_type()) {
      case CommandType::Message:
        cursor.advance(handle_message(context, cursor, arb));
        break;
      case CommandType::Transaction:
        cursor.advance(handle_transaction(context, cursor, arb));
        break;
      case CommandType::Invalid:
        halt = true;
        break;
      default:
        break;
    }
  } while (!halt && epoch.in_interval(cursor.time()));

  set_time(cursor.time());
}

std::size_t Agent::handle_message(Context & context,
                                  const Cursor & cursor,
                                  CommandArbitrator & arb) {
  const TimeStamped<Message*> tsm{mq_.front()};
  const CoherenceActions actions{get_actions(context, cursor, tsm.t())};

  const Message * msg = tsm.t();
  Cursor mcur{cursor};
  switch (actions.result()) {
    case MessageResult::Commit:
      execute(context, mcur, actions, msg);
      mq_.pop_front();
      // TODO: refactor
      if (actions.transaction_done())
        trns_->event(TransactionEvent::End,
                     TimeStamped{mcur.time(), msg->transaction()});
      msg->release();
      break;
    case MessageResult::Stall:
      arb.disable_message_class(MessageType::to_class(msg->type()));
      break;
  }
  return actions.cost();
}

std::size_t Agent::handle_transaction(Context & context,
                                      const Cursor & cursor,
                                      CommandArbitrator & arb) {
  const TimeStamped<Transaction *> tst{tq_.front()};
  const Transaction * t = tst.t();
  
  const CoherenceActions actions{get_actions(context, cursor, t)};
  if (actions.requires_eviction()) {
    // In this simple model, a transaction issue fails whenever
    // the transaction would require a eviction of a previously
    // installed line in the cache. To handle this case, the
    // current transaction is aborted, time is advanced (to
    // account for the lookup overhead, a new 'replacement'
    // transaction is inserted on the head of the transaction
    // queue and the transaction replayed.
    //
    log_debug("Transaction caused EVICTION: ", to_string(*t));

    // Create new replacement transaction to the current address
    // and place at the head of the queue manager.
    //
    enqueue_replacement(cursor, t);
  } else {
    // TODO: this should really be associated with the transaction
    // itself, not the transaction source.
    if ((t->type() == TransactionType::load) ||
        (t->type() == TransactionType::store))
      trns_->event(TransactionEvent::Start, TimeStamped{cursor.time(), t});
    
    bool do_commit{true};
    switch (actions.result()) {
      case TransactionResult::Blocked:
        // The transaction may not advance because there are pending
        // operations on the line.
        //
        do_commit = false;
        break;

      case TransactionResult::Miss:
        log_debug("Transaction MISS: ", to_string(*t));
        ++stats_.misses_n;
        break;
      case TransactionResult::Hit:
        log_debug("Transaction HIT: ", to_string(*t));
        ++stats_.hits_n;
        break;
    }
    if (do_commit) {
      Cursor tcur{cursor};
      execute(context, tcur, actions, t);
      tq_.pop_front();
    } else {
      arb.disable_transactions();
    }
  }
  return actions.cost();
}

void Agent::enqueue_replacement(const Cursor & cursor, const Transaction * t) {
  Transaction * trpl = tfac_.construct();
  trpl->set_type(TransactionType::replacement);
  trpl->set_addr(t->addr());
  tq_.set_replacement(TimeStamped{cursor.time(), trpl});
}

void Agent::fetch_transactions(std::size_t n) {
  TimeStamped<Transaction*> ts;
  for (int i = 0; i < n; i++) {
    if (!trns_->get_transaction(ts)) break;

    tq_.push_back(ts);
  }
}

CoherenceActions Agent::get_actions(
    Context& context, const Cursor& cursor, const Message *msg) {
  const Transaction * t = msg->transaction();
  CCM_AGENT_ASSERT(cache_->is_hit(t->addr()));
  CacheLine& cache_line = cache_->lookup(t->addr());
  CoherenceActions actions = cc_model_->get_actions(msg, cache_line);
  CCM_AGENT_ASSERT(!actions.error());
  return actions;
}

CoherenceActions Agent::get_actions(
    Context& context, const Cursor& cursor, const Transaction * t) {
  CoherenceActions actions;
  if (t->type() != TransactionType::replacement)
    actions.set_requires_eviction(cache_->requires_eviction(t->addr()));

  if (!actions.requires_eviction()) {
    if (!cache_->is_hit(t->addr())) {
      CacheLine cache_line;
      cc_model_->init(cache_line);
      cache_->install(t->addr(), cache_line);
    }
    CacheLine & cache_line = cache_->lookup(t->addr());
    actions = cc_model_->get_actions(t, cache_line);
    actions.set_cost(CoherenceActions::compute_cost(actions));
  } else {
    actions.set_requires_eviction(false);
#define CACHE_LOOKUP_PENALTY 1
    actions.set_cost(CACHE_LOOKUP_PENALTY);
  }
  return actions;
}

void Agent::apply(TimeStamped<Message*> ts) { mq_.push_back(ts); }

bool Agent::is_active() const {
  // Agent is active whenever there are pending messages in the
  // associated message queue manager, or when there are either
  // transactions in the transaction manager, or transaction that are
  // yet to be issued.
  //
  return (tq_.is_active() || mq_.is_active() || trns_->is_active());
}
#ifdef ENABLE_JSON

std::unique_ptr<Agent> AgentBuilder::construct(
    const Platform & platform, LoggerScope * l, nlohmann::json j) {
  std::unique_ptr<Agent> agent =
      std::make_unique<Agent>(AgentOptions::from_json(platform, l, j));
  agent->set_transaction_source(
      TransactionSource::from_json(j["transaction_source"]));
  return std::move(agent);
}
#endif
}  // namespace ccm
