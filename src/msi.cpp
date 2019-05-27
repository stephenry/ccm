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

#include "msi.hpp"
#include <algorithm>
#include <array>
#include "actor.hpp"
#include "agent.hpp"
#include "common.hpp"
#include "message.hpp"
#include "snoopfilter.hpp"
#include "transaction.hpp"

namespace ccm {

const char *MsiAgentLineState::to_string(state_t s) {
  // clang-format off
  switch (s) {
#define __declare_to_string(__e) \
  case MsiAgentLineState::__e:   \
    return #__e;
    MSI_LINE_STATES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
  // clang-format on
}

bool MsiAgentLineState::is_stable(state_t s) {
  switch (s) {
    case MsiAgentLineState::M:
    case MsiAgentLineState::S:
    case MsiAgentLineState::I:
      return true;
    default:
      return false;
  }
}

struct MsiAgentProtocol::MsiAgentProtocolImpl {
  MsiAgentProtocolImpl(const Platform & platform) : platform_(platform) {}

  void init(CacheLine &l) const {
    l.set_state(MsiAgentLineState::I);
  }

  bool is_stable(const CacheLine &l) const {
    return MsiAgentLineState::is_stable(l.state());
  }

  std::string to_string(state_t s) const {
    return MsiAgentLineState::to_string(s);
  }

  CoherenceActions get_actions(const Transaction *t,
                               const CacheLine &cache_line) const {
    CoherenceActions actions;
    switch (t->type()) {
      case TransactionType::load:
        handle_load(t, cache_line, actions);
        break;
      case TransactionType::store:
        handle_store(t, cache_line, actions);
        break;
      case TransactionType::replacement:
        handle_replacement(t, cache_line, actions);
        break;
      default:
        actions.set_error(true);
        break;
    }
    return actions;
  }

  CoherenceActions get_actions(const Message *m,
                               const CacheLine &cache_line) const {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::FwdGetS:
        handle_fwd_gets(m, cache_line, actions);
        break;

      case MessageType::FwdGetM:
        handle_fwd_getm(m, cache_line, actions);
        break;

      case MessageType::Inv:
        handle_inv(m, cache_line, actions);
        break;

      case MessageType::PutS:
      case MessageType::PutM:
        handle_put_ack(m, cache_line, actions);
        break;

      case MessageType::Data:
        handle_data(m, cache_line, actions);
        break;

      default:
        actions.set_error(true);
    }
    return actions;
  }

  bool message_requires_eviction(const Message *m) { return false; }

  void handle_load(const Transaction *t, const CacheLine &cache_line,
                    CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::I:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.append_command(CoherentAgentCommand::EmitGetS);
        a.set_next_state(MsiAgentLineState::IS_D);
        a.set_result(TransactionResult::Miss);
        break;

      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MsiAgentLineState::S:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_store(const Transaction *t, const CacheLine &cache_line,
                     CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::I:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.set_next_state(MsiAgentLineState::IM_AD);
        a.set_result(TransactionResult::Miss);
        break;

      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MsiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::SM_AD);
        a.set_result(TransactionResult::Miss);
        break;

      case MsiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_replacement(const Transaction *t, const CacheLine &cache_line,
                           CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitPutS);
        a.set_next_state(MsiAgentLineState::SI_A);
        a.set_result(TransactionResult::Consumed);
        break;

      case MsiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitPutM);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.set_next_state(MsiAgentLineState::MI_A);
        a.set_result(TransactionResult::Consumed);
        break;
      
      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        // Cannot evict a line for which there are already messages
        // in- flight.
        a.set_result(TransactionResult::Blocked);
        break;
      
      default:
        // State: [I]; cannot evict a line that is not installed in
        // the cache.
        a.set_error(true);
    }
  }

  void handle_fwd_gets(const Message *m, const CacheLine &cache_line,
                       CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MsiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::S);
        a.set_result(MessageResult::Commit);
        break;

      case MsiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::EmitDataToDir);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::SI_A);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_fwd_getm(const Message *m, const CacheLine &cache_line,
                       CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MsiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::S);
        a.set_result(MessageResult::Commit);
        break;

      case MsiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::SI_A);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_inv(const Message *m, const CacheLine &cache_line,
                   CoherenceActions &a) const {
    if (m->is_ack()) {
      const bool is_last_ack = cache_line.is_last_inv_ack();

      a.append_command(CoherentAgentCommand::IncAckCount);

      if (is_last_ack) {
        switch (cache_line.state()) {
          case MsiAgentLineState::IM_A:
          case MsiAgentLineState::SM_A:
            a.append_command(CoherentAgentCommand::UpdateState);
            a.set_next_state(MsiAgentLineState::M);
            a.set_result(MessageResult::Commit);
            break;

          default:
            a.set_error(true);
        }
      }

    } else {
      // Inbound invalidation request from the home directory. Update
      // line state accordingly.
      //
      switch (cache_line.state()) {
        case MsiAgentLineState::IS_D:
          a.set_result(MessageResult::Stall);
          break;

        case MsiAgentLineState::S:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::I);
          a.set_result(MessageResult::Commit);
          break;

        case MsiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::IM_AD);
          a.set_result(MessageResult::Commit);
          break;

        case MsiAgentLineState::SI_A:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::II_A);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
      }
    }
  }

  void handle_put_ack(const Message *m, const CacheLine &cache_line,
                      CoherenceActions &a) const {
    switch (cache_line.state()) {
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MsiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_data(const Message *m, const CacheLine &cache_line,
                    CoherenceActions &a) const {
    // The snoop filter advertises that this Data is unique and that
    // the agent need not block awaiting invalidation acknowledgements
    // for other agents in the system.
    //
    const bool is_last_ack = (m->ack_count() == 0);

    if (is_last_ack) {
      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).n
      //
      switch (cache_line.state()) {
        case MsiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::S);
          a.set_transaction_done(true);
          a.set_result(MessageResult::Commit);
          break;

        case MsiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::M);
          a.set_result(MessageResult::Commit);
          break;

        case MsiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::M);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
      }
    } else {
      // Data has been received, but we are currently awaiting
      // pending responses from other agents in the system.
      //
      switch (cache_line.state()) {
        case MsiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::SetAckExpectCount);
          a.set_ack_count(m->ack_count());
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::IM_A);
          a.set_result(MessageResult::Commit);
          break;

        case MsiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::SetAckExpectCount);
          a.set_ack_count(m->ack_count());
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::SM_A);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
      }
    }
  }

  const Platform platform_;
};

MsiAgentProtocol::MsiAgentProtocol(const Platform & platform) {
  impl_ = std::make_unique<MsiAgentProtocolImpl>(platform);
}

MsiAgentProtocol::~MsiAgentProtocol(){};

void MsiAgentProtocol::init(CacheLine &l) const { impl_->init(l); }

bool MsiAgentProtocol::is_stable(const CacheLine &l) const {
  return impl_->is_stable(l);
}

std::string MsiAgentProtocol::to_string(state_t s) const {
  return impl_->to_string(s);
}

CoherenceActions MsiAgentProtocol::get_actions(
    const Transaction *t, const CacheLine &cache_line) const {
  return impl_->get_actions(t, cache_line);
}

CoherenceActions MsiAgentProtocol::get_actions(
    const Message *m, const CacheLine &cache_line) const {
  return impl_->get_actions(m, cache_line);
}

const char *MsiDirectoryLineState::to_string(state_t s) {
  switch (s) {
#define __declare_to_string(__e)   \
  case MsiDirectoryLineState::__e: \
    return #__e;
    MSI_DIRECTORY_STATES(__declare_to_string)
#undef __declare_to_string
  }
  return "<Invalid Directory State>";
}

struct MsiSnoopFilterProtocol::MsiSnoopFilterProtocolImpl {
  MsiSnoopFilterProtocolImpl() {}

  void init(DirectoryLine &l) const { l.set_state(MsiDirectoryLineState::I); }

  bool is_stable(const DirectoryLine &l) const { return true; }

  std::string to_string(const DirectoryLine &l) const {
    std::stringstream ss;
    ss << MsiDirectoryLineState::to_string(l.state());
    return ss.str();
  }

  std::string to_string(state_t l) const {
    return MsiDirectoryLineState::to_string(l);
  }

  CoherenceActions get_actions(const Message *m,
                               const DirectoryLine &dir_entry) {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::GetS:
        handle_gets(m, dir_entry, actions);
        break;

      case MessageType::GetM:
        handle_getm(m, dir_entry, actions);
        break;

      case MessageType::PutS:
        handle_puts(m, dir_entry, actions);
        break;

      case MessageType::PutM:
        handle_putm(m, dir_entry, actions);
        break;

      case MessageType::Data:
        handle_data(m, dir_entry, actions);
        break;

      default:
        actions.set_error(true);
    }
    return actions;
  }

 private:
  void handle_gets(const Message *m, const DirectoryLine &dir_entry,
                    CoherenceActions &a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.set_ack_count(0);
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;

      case MsiDirectoryLineState::S:
        a.set_ack_count(0);
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;

      case MsiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::AddOwnerToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;

      case MsiDirectoryLineState::S_D:
        a.set_result(MessageResult::Stall);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_getm(const Message *m, const DirectoryLine &dir_entry,
                    CoherenceActions &a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_ack_count(0);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;

      case MsiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_ack_count(dir_entry.num_sharers_not_id(m->src_id()));
        a.append_command(SnoopFilterCommand::SendInvToSharers);
        a.append_command(SnoopFilterCommand::ClearSharers);
        a.append_command(SnoopFilterCommand::SetOwnerToReq);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;

      case MsiDirectoryLineState::M:
        break;

      case MsiDirectoryLineState::S_D:
        a.set_result(MessageResult::Stall);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_puts(const Message *m, const DirectoryLine &dir_entry,
                    CoherenceActions &a) const {
    const bool is_last = false;  // TODO
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MsiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        if (is_last) {
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;

      case MsiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MsiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_putm(const Message *m, const DirectoryLine &dir_entry,
                    CoherenceActions &a) const {
    const bool is_data_from_owner = false;  // TODO
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        if (!is_data_from_owner) {
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;

      case MsiDirectoryLineState::S:
        if (!is_data_from_owner) {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;

      case MsiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (is_data_from_owner) {
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;

      case MsiDirectoryLineState::S_D:
        if (!is_data_from_owner) {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;

      default:
        a.set_error(true);
    }
  }

  void handle_data(const Message *m, const DirectoryLine &dir_entry,
                    CoherenceActions &a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::CpyDataToMemory);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;

      default:
        a.set_error(true);
    }
  }

  bool message_requires_recall(const Message *m) { return false; }

  bool transaction_must_hit(MessageType t) { return true; }
};

MsiSnoopFilterProtocol::MsiSnoopFilterProtocol() {
  impl_ = std::make_unique<MsiSnoopFilterProtocolImpl>();
}

MsiSnoopFilterProtocol::~MsiSnoopFilterProtocol() {}

void MsiSnoopFilterProtocol::init(DirectoryLine &l) const { impl_->init(l); }

bool MsiSnoopFilterProtocol::is_stable(const DirectoryLine &l) const {
  return impl_->is_stable(l);
}

std::string MsiSnoopFilterProtocol::to_string(const DirectoryLine &l) const {
  return impl_->to_string(l);
}

std::string MsiSnoopFilterProtocol::to_string(state_t l) const {
  return impl_->to_string(l);
}

CoherenceActions MsiSnoopFilterProtocol::get_actions(
    const Message *m, const DirectoryLine &dir_entry) const {
  return impl_->get_actions(m, dir_entry);
}

MsiCoherenceProtocolValidator::MsiCoherenceProtocolValidator() {}

bool MsiCoherenceProtocolValidator::validate_addr(
    addr_t addr, const std::vector<Entry<CacheLine> > &lines,
    const DirectoryLine &entry) const {
  bool pass = true;

  std::array<std::size_t, MsiAgentLineState::STATE_COUNT> state_count;
  std::fill(state_count.begin(), state_count.end(), 0);

  for (const Entry<CacheLine> &cl : lines) {
    const CacheLine &c = std::get<1>(cl);

    state_count[c.state()]++;
  }

  if ((state_count[MsiAgentLineState::M] != 1) &&
      (state_count[MsiAgentLineState::M] != 0)) {
    pass = false;

    error("Multiple MODIFIED agents are present.");
  }

  if ((state_count[MsiAgentLineState::M] > 0) &&
      (state_count[MsiAgentLineState::S] != 0)) {
    pass = false;

    error("When MODIFIED is present, other SHARED agents cannot be present.");
  }
  return pass;
}

}  // namespace ccm
