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
#include "agent.hpp"
#include "snoopfilter.hpp"
#include "message.hpp"
#include "transaction.hpp"
#include "actors.hpp"
#include "common.hpp"
#include <algorithm>

namespace ccm {

const char * to_string(MsiAgentLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiAgentLineState::__e: return #__e;
    MSI_LINE_STATES(__declare_to_string)
#undef __declare_to_string
    default: return "<Invalid Line State>";
  }
}

CacheLine::state_type _g(MsiAgentLineState s) {
  return static_cast<CacheLine::state_type>(s);
}

namespace {

MsiAgentLineState _s(CacheLine::state_type s) {
  return static_cast<MsiAgentLineState>(s);
}

} // namespace

bool is_stable(MsiAgentLineState s) {
  switch (s) {
    case MsiAgentLineState::M:
    case MsiAgentLineState::S:
    case MsiAgentLineState::I:
      return true;
    default:
      return false;
  }
}

struct MsiCoherentAgentModel::MsiCoherentAgentModelImpl {
  MsiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts)
  {}

  void init(CacheLine & l) const {
    l.set_state(static_cast<CacheLine::state_type>(MsiAgentLineState::I));
  }
  
  bool is_stable(const CacheLine & l) const {
    return ::ccm::is_stable(_s(l.state()));
  }

  std::string to_string(CacheLine::state_type s) const {
    return ::ccm::to_string(static_cast<MsiAgentLineState>(s));
  }

  CoherenceActions get_actions(
      Transaction * t, const CacheLine & cache_line) const {
    CoherenceActions actions;
    switch (t->type()) {
      case TransactionType::Load:
        handle__Load(t, cache_line, actions);
        break;
        
      case TransactionType::Store:
        handle__Store(t, cache_line, actions);
        break;
    }
    return actions;
  }
  
  CoherenceActions get_actions(
      const Message * m, const CacheLine & cache_line) const {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::FwdGetS:
        handle__FwdGetS(m, cache_line, actions);
        break;

      case MessageType::FwdGetM:
        handle__FwdGetM(m, cache_line, actions);
        break;

      case MessageType::Inv:
        handle__Inv(m, cache_line, actions);
        break;

      case MessageType::PutS:
      case MessageType::PutM:
        handle__PutAck(m, cache_line, actions);
        break;

      case MessageType::Data:
        handle__Data(m, cache_line, actions);
        break;

      default:
        actions.set_error(true);
    }
    return actions;
  }

  bool message_requires_eviction(const Message * m) {
    return false;
  }

  void handle__Load(
      Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (_s(cache_line.state())) {
      case MsiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetS);
        a.append_command(CoherentAgentCommand::UpdateState);
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
        break;
        
      default:
        a.set_error(true);
    }
  }
  
  void handle__Store(
      Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {
    switch (_s(cache_line.state())) {
      case MsiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
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
        break;
        
      default:
        a.set_error(true);
    }
  }

  void handle__FwdGetS(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {
    switch (_s(cache_line.state())) {
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

  void handle__FwdGetM(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {
    switch (_s(cache_line.state())) {
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

  void handle__Inv(
      const Message *m, const CacheLine & cache_line, CoherenceActions & a) const {
    if (m->is_ack()) {
      const bool is_last_ack = (cache_line.ack_count() == 1);

      a.append_command(CoherentAgentCommand::SetAckCount);
      a.set_ack_count(cache_line.ack_count() - 1);
      
      if (is_last_ack) {
        switch (_s(cache_line.state())) {
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
      switch (_s(cache_line.state())) {
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

  void handle__PutAck(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {
    switch (_s(cache_line.state())) {
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

  void handle__Data(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    const bool is_last_ack = (m->ack_count() == 0);

    if (is_last_ack) {

      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).n
      //
      switch (_s(cache_line.state())) {
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
      switch (_s(cache_line.state())) {
        case MsiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::SetAckCount);
          a.set_ack_count(m->ack_count());
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MsiAgentLineState::IM_A);
          a.set_result(MessageResult::Commit);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::SetAckCount);
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

  const CoherentAgentOptions opts_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

void MsiCoherentAgentModel::init(CacheLine & l) const {
  impl_->init(l);
}

bool MsiCoherentAgentModel::is_stable(const CacheLine & l) const {
  return impl_->is_stable(l);
}

std::string MsiCoherentAgentModel::to_string(CacheLine::state_type s) const {
  return impl_->to_string(s);
}

CoherenceActions MsiCoherentAgentModel::get_actions(
    Transaction * t, const CacheLine & cache_line) const {
  return impl_->get_actions(t, cache_line);
}

CoherenceActions MsiCoherentAgentModel::get_actions(
    const Message * m, const CacheLine & cache_line) const {
  return impl_->get_actions(m, cache_line);
}

const char * to_string(MsiDirectoryLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiDirectoryLineState::__e: return #__e;
    MSI_DIRECTORY_STATES(__declare_to_string)
#undef __declare_to_string
  }
  return "<Invalid Directory State>";
}

CacheLine::state_type _g(MsiDirectoryLineState s) {
  return static_cast<CacheLine::state_type>(s);
}

struct MsiSnoopFilterModel::MsiSnoopFilterModelImpl {
  
  MsiSnoopFilterModelImpl(const SnoopFilterOptions & opts)
      : opts_(opts) 
  {}

  void init(DirectoryEntry & l) const {
    l.set_state(
        static_cast<DirectoryEntry::state_type>(MsiDirectoryLineState::I));
  }
  
  bool is_stable(const DirectoryEntry & l) const {
    return true;
  }
  
  std::string to_string(const DirectoryEntry & l) const {
    std::stringstream ss;
    ss << ::ccm::to_string(static_cast<MsiDirectoryLineState>(l.state()));
    return ss.str();
  }
  
  std::string to_string(const MsiDirectoryLineState l) const {
    return ::ccm::to_string(l);
    
  }

  CoherenceActions get_actions(
      const Message * m, const DirectoryEntry & dir_entry) {
    CoherenceActions actions;
    switch (m->type()) {
      case MessageType::GetS:
        handle__GetS(m, dir_entry, actions);
        break;
          
      case MessageType::GetM:
        handle__GetM(m, dir_entry, actions);
        break;
          
      case MessageType::PutS:
        handle__PutS(m, dir_entry, actions);
        break;
          
      case MessageType::PutM:
        handle__PutM(m, dir_entry, actions);
        break;
          
      case MessageType::Data:
        handle__Data(m, dir_entry, actions);
        break;
          
      default:
        actions.set_error(true);
    }
    return actions;
  }
 private:
  
  void handle__GetS(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {
    switch (static_cast<MsiDirectoryLineState>(dir_entry.state())) {
      case MsiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_ack_count(0);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;

      case MsiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.set_ack_count(0);
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
        //        a.set_stall();
        break;

      default:
        a.set_error(true);
    }
  }

  void handle__GetM(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {
    switch (static_cast<MsiDirectoryLineState>(dir_entry.state())) {
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
        //        a.set_stall();
        break;

      default:
        a.set_error(true);
    }
  }

  void handle__PutS(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {
    const bool is_last = false; // TODO
    switch (static_cast<MsiDirectoryLineState>(dir_entry.state())) {
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

  void handle__PutM(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {
    const bool is_data_from_owner = false;
    switch (static_cast<MsiDirectoryLineState>(dir_entry.state())) {
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

  void handle__Data(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {
    switch (static_cast<MsiDirectoryLineState>(dir_entry.state())) {
      case MsiDirectoryLineState::S_D:
        a.append_command(SnoopFilterCommand::CpyDataToMemory);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;

      default:
        a.set_error(true);
    }
  }

  bool message_requires_recall(const Message * m) {
    return false;
  }

  bool transaction_must_hit(MessageType t) {
    return true;
  }
  
  const SnoopFilterOptions opts_;
};

MsiSnoopFilterModel::MsiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
  impl_ = std::make_unique<MsiSnoopFilterModelImpl>(opts);
}

MsiSnoopFilterModel::~MsiSnoopFilterModel() {}

void MsiSnoopFilterModel::init(DirectoryEntry & l) const {
  impl_->init(l);
}

bool MsiSnoopFilterModel::is_stable(const DirectoryEntry & l) const {
  return impl_->is_stable(l);
}

std::string MsiSnoopFilterModel::to_string(const DirectoryEntry & l) const {
  return impl_->to_string(l);
}

std::string MsiSnoopFilterModel::to_string(CacheLine::state_type l) const {
  return impl_->to_string(static_cast<MsiDirectoryLineState>(l));
}

CoherenceActions MsiSnoopFilterModel::get_actions(
    const Message * m, const DirectoryEntry & dir_entry) const {
  return impl_->get_actions(m, dir_entry);
}

} // namespace ccm
