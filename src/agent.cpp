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

#define MESSAGE_ACTION_COST 1
#define TRANSACTION_ACTION_COST 1

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

CoherentAgentCommandInvoker::CoherentAgentCommandInvoker(
    const CoherentAgentOptions& opts)
    : CoherentActor(opts), msgd_(opts) {
  cc_model_ = agent_protocol_factory(opts.protocol(), opts.platform());
  cache_ = cache_factory<CacheLine>(opts.cache_options());
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
    log_debug(cursor.time(), "Execute: ", CoherentAgentCommand::to_string(cmd));

    switch (cmd) {
      case CoherentAgentCommand::UpdateState:
        execute_update_state(cursor, t, actions.next_state());
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
    log_debug(cursor.time(), "Execute: ", CoherentAgentCommand::to_string(cmd));

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
        execute_update_state(cursor, t, actions.next_state());
        break;
      case CoherentAgentCommand::IncAckCount:
        execute_inc_ack_count(cursor, t);
        break;
      case CoherentAgentCommand::SetAckExpectCount:
        execute_set_ack_expect_count(cursor, msg);
        break;
      default:
        break;
    }
    cursor.advance(CoherentAgentCommand::to_cost(cmd));
  }
}

void CoherentAgentCommandInvoker::execute_update_state(Cursor & cursor,
                                                       const Transaction* t,
                                                       state_t next_state) {
  CacheLine & cache_line = cache_->lookup(t->addr());
  
  log_debug(cursor.time(), "Update state; current: ", cc_model_->to_string(next_state),
            " previous: ", cc_model_->to_string(cache_line.state()));
  cache_line.set_state(next_state);
}

void CoherentAgentCommandInvoker::execute_emit_gets(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::GetS);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Sending GetS to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_getm(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::GetM);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Sending GetM to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_puts(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutS);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Sending PutS to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_pute(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutE);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Sending PutE to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_puto(Context& context,
                                                    Cursor& cursor,
                                                    const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::PutO);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Sending PutO to home directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_data_to_dir(Context& context,
                                                           Cursor& cursor,
                                                           const Transaction* t) {
  const Platform platform = opts_.platform();
  MessageBuilder b = msgd_.builder();

  b.set_src_id(id());
  b.set_type(MessageType::Data);
  b.set_dst_id(platform.get_snoop_filter_id(t->addr()));
  b.set_transaction(t);

  Message *msg = b.msg();
  log_debug(cursor.time(), "Emit Data To Directory: ", to_string(*msg));
  context.emit_message(TimeStamped{cursor.time(), msg});
}

void CoherentAgentCommandInvoker::execute_emit_data_to_req(Context& context,
                                                           Cursor& cursor,
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
  log_debug(cursor.time(), "Emit Data To Requester: ", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});
}

void CoherentAgentCommandInvoker::execute_emit_inv_ack(Context& context,
                                                       Cursor& cursor,
                                                       const Message* msg) {
  MessageBuilder b = msgd_.builder();
  b.set_src_id(id());
  b.set_type(MessageType::Inv);
  b.set_is_ack(true);
  b.set_dst_id(msg->fwd_id());
  b.set_transaction(msg->transaction());

  Message *out = b.msg();
  log_debug(cursor.time(), "Sending invalidation acknowledgement.", to_string(*out));
  context.emit_message(TimeStamped{cursor.time(), out});
}

void CoherentAgentCommandInvoker::execute_inc_ack_count(Cursor & cursor,
                                                        const Transaction * t) {
  CacheLine & cache_line = cache_->lookup(t->addr());
  cache_line.set_inv_ack_count(cache_line.inv_ack_count() + 1);
  log_debug(cursor.time(), "Update invalidation count: ", cache_line.inv_ack_count());
}

void CoherentAgentCommandInvoker::execute_set_ack_expect_count(
    Cursor & cursor, const Message * msg) {
  const Transaction * t = msg->transaction();
  CacheLine & cache_line = cache_->lookup(t->addr());
  cache_line.set_inv_ack_expect_valid(true);
  cache_line.set_inv_ack_expect(msg->ack_count());
  log_debug(cursor.time(), "Update expected invalidation count: ",
            cache_line.inv_ack_expect());
}

Agent::CommandArbitrator::~CommandArbitrator() {}

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

  CommandArbitrator arb{tq_, mq_};
  do {
    arb.arbitrate();

    if (!arb.is_valid() || !epoch.in_interval(arb.frontier())) break;
    
    switch (arb.command_type()) {
      case CommandType::Message: {
        const TimeStamped<Message*> tsm{mq_.front()};
        Message * msg = tsm.t();
        
        switch (handle_message(context, cursor, msg)) {
          case MessageResult::Stall:
            arb.disable_message_class(MessageType::to_class(msg->type()));
            break;
          case MessageResult::Commit:
            mq_.pop_front();
            msg->release();
            break;
        }
      } break;
      case CommandType::Transaction: {
        switch (handle_transaction(context, cursor, arb)) {
          case TransactionResult::Miss:
            ++stats_.misses_n;
            break;
          case TransactionResult::Hit:
            ++stats_.hits_n;
            break;
          case TransactionResult::Consumed:
          case TransactionResult::Blocked:
            break;
        }
      } break;
      default:
        break;
    }
  } while (epoch.in_interval(cursor.time()));

  set_time(std::max(cursor.time(), epoch.end()));
}

result_t Agent::handle_message(Context & context, Cursor & cursor,
                               const Message * msg) {
  const CoherenceActions actions{get_actions(context, cursor, msg)};
  if (actions.result() == MessageResult::Commit) {
    execute(context, cursor, actions, msg);
    if (actions.transaction_done()) {
      const Transaction * t = msg->transaction();

      log_debug(cursor.time(), "Transaction TID=", t->tid(), " END");
      t->event(TransactionEvent::End, cursor.time());
    }
  }
  return actions.result();
}

result_t Agent::handle_transaction(Context & context, Cursor & cursor,
                                   CommandArbitrator & arb) {
  const TimeStamped<Transaction *> tst{tq_.front()};
  const Transaction * t = tst.t();
  
  if (t->type() != TransactionType::replacement) {
    log_debug(cursor.time(), "Transaction TID=", t->tid(), " START");
    t->event(TransactionEvent::Start, cursor.time());
  }
  
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
    log_debug(cursor.time(), "Transaction caused EVICTION: ", to_string(*t));

    // Create new replacement transaction to the current address
    // and place at the head of the queue manager.
    //
    enqueue_replacement(cursor, t);
  } else {
    const bool do_commit = (actions.result() != TransactionResult::Blocked);
    if (do_commit) {
      execute(context, cursor, actions, t);
      tq_.pop_front();
      if (actions.transaction_done()) {
        log_debug(cursor.time(), "Transaction TID=", t->tid(), " END");
        t->event(TransactionEvent::End, cursor.time());
      }
    } else {
      arb.disable_transactions();
    }
  }
  return actions.result();
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
    Context& context, Cursor& cursor, const Message *msg) {
  cursor.advance(MESSAGE_ACTION_COST);
  
  const Transaction * t = msg->transaction();
  CCM_AGENT_ASSERT(cache_->is_hit(t->addr()));
  CacheLine& cache_line = cache_->lookup(t->addr());
  CoherenceActions actions = cc_model_->get_actions(msg, cache_line);
  CCM_AGENT_ASSERT(!actions.error());
  return actions;
}

CoherenceActions Agent::get_actions(
    Context& context, Cursor& cursor, const Transaction * t) {
  cursor.advance(TRANSACTION_ACTION_COST);
  
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
