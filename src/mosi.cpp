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

#include "mosi.hpp"

namespace ccm {

const char * MosiAgentLineState::to_string(state_t state) {
  switch (state) {
#define __declare_state(__name)                                 \
    case MosiAgentLineState::__name: return #__name; break;
    MOSI_LINE_STATES(__declare_state)
#undef __declare_state
    default: return "Invalid State";
  }
}

bool MosiAgentLineState::is_stable(state_t state) {
  bool ret{false};
  switch (state) {
  case MosiAgentLineState::I:
  case MosiAgentLineState::S:
  case MosiAgentLineState::O:
  case MosiAgentLineState::M:
    ret = true;
    break;

  default:
    ret = false;
    break;
  }
  return ret;
}
  
const char * MosiDirectoryLineState::to_string(state_t state) {
  switch (state) {
#define __declare_state(__name)                                 \
  case MosiDirectoryLineState::__name: return #__name; break;
    MOSI_DIRECTORY_STATES(__declare_state)
#undef __declare_state
  default: return "Invalid State";
  }
}

bool MosiDirectoryLineState::is_stable(state_t state) {
  bool ret{false};
  switch (state) {
  case MosiDirectoryLineState::I:
  case MosiDirectoryLineState::S:
  case MosiDirectoryLineState::O:
  case MosiDirectoryLineState::M:
    ret = true;
    break;
    
  default:
    ret = false;
    break;
  }
  return ret;
}

struct MosiCoherentAgentModel::MosiCoherentAgentModelImpl {
  MosiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts)
  {}

  void init(CacheLine & cache_line) const {
    cache_line.set_state(MosiAgentLineState::I);
  }

  bool is_stable(state_t state) const {
    return MosiAgentLineState::is_stable(state);
  }

  std::string to_string(state_t state) const {
    return MosiAgentLineState::to_string(state);
  }

  CoherenceActions get_actions(
      const Transaction * t, const CacheLine & cache_line) const {
    CoherenceActions actions;
    switch (t->type()) {
      case TransactionType::Load:
        handle__Load(t, cache_line, actions);
        break;
        
      case TransactionType::Store:
        handle__Store(t, cache_line, actions);
        break;

      default:
        actions.set_error(true);
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

      case MessageType::AckCount:
        handle__AckCount(m, cache_line, actions);

      default:
        actions.set_error(true);
        break;
    }
    return actions;
  }

private:
  void handle__Load(
      const Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetS);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IS_D);
        a.set_result(TransactionResult::Miss);
        break;
        
      case MosiAgentLineState::IS_D:
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::S:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
      case MosiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
        
      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::O:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;
        
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;

      default:
        a.set_error(true);
        break;
    }
  }
  
  void handle__Store(
      const Transaction * t, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::I:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IM_AD);
        a.set_result(TransactionResult::Miss);
        break;

      case MosiAgentLineState::IS_D:
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::S:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::SM_AD);
        a.set_result(TransactionResult::Miss);
        break;

      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::M:
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;

      case MosiAgentLineState::MI_A:
        a.set_result(TransactionResult::Blocked);
        break;

      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitGetM);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OM_AC);
        a.set_result(TransactionResult::Hit);
        a.set_transaction_done(true);
        break;

      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.set_result(TransactionResult::Blocked);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  void handle__FwdGetS(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MosiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::O);
        a.set_result(MessageResult::Commit);
        break;

      case MosiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::OI_A);
        a.set_result(MessageResult::Commit);
        break;
      
      case MosiAgentLineState::O:
      case MosiAgentLineState::OM_AC:
      case MosiAgentLineState::OM_A:
      case MosiAgentLineState::OI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.set_result(MessageResult::Commit);
        break;
      
      default:
        a.set_error(true);
        break;
    }
  }
  

  void handle__FwdGetM(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::IM_AD:
      case MosiAgentLineState::IM_A:
      case MosiAgentLineState::SM_AD:
      case MosiAgentLineState::SM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MosiAgentLineState::M:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;

      case MosiAgentLineState::MI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::II_A);
        a.set_result(MessageResult::Commit);
        break;

      case MosiAgentLineState::O:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;

      case MosiAgentLineState::OM_AC:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::IM_AD);
        a.set_result(MessageResult::Commit);
        break;

      case MosiAgentLineState::OM_A:
        a.set_result(MessageResult::Stall);
        break;

      case MosiAgentLineState::OI_A:
        a.append_command(CoherentAgentCommand::EmitDataToReq);
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::II_A);
        a.set_result(MessageResult::Commit);
        break;
        
      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Inv(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    if (m->is_ack()) {
      const bool is_last_ack = (cache_line.ack_count() == 1);

      a.append_command(CoherentAgentCommand::SetAckCount);
      a.set_ack_count(cache_line.ack_count() - 1);

      if (is_last_ack) {
      
        switch (cache_line.state()) {
          case MosiAgentLineState::IM_A:
          case MosiAgentLineState::SM_A:
          case MosiAgentLineState::OM_A:
            a.append_command(CoherentAgentCommand::UpdateState);
            a.set_next_state(MosiAgentLineState::M);
            break;

          default:
            a.set_error(true);
            break;
        }

      }      

    } else {
      // Invalidation request
      switch (cache_line.state()) {
        case MosiAgentLineState::IS_D:
          a.set_result(MessageResult::Stall);
          break;
          
        case MosiAgentLineState::S:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::I);
          a.set_result(MessageResult::Commit);
          break;
          
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::IM_AD);
          a.set_result(MessageResult::Commit);
          break;

        case MosiAgentLineState::SI_A:
          a.append_command(CoherentAgentCommand::EmitInvAck);
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::II_A);
          a.set_result(MessageResult::Commit);
          break;

        default:
          a.set_error(true);
          break;
      }
          
    }
  }
  
  void handle__PutAck(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::MI_A:
      case MosiAgentLineState::OI_A:
      case MosiAgentLineState::SI_A:
      case MosiAgentLineState::II_A:
        a.append_command(CoherentAgentCommand::UpdateState);
        a.set_next_state(MosiAgentLineState::I);
        a.set_result(MessageResult::Commit);
        break;
      
      default:
        a.set_error(true);
        break;
    }
  }

  void handle__Data(
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    const Platform platform = opts_.platform();
    const bool is_from_dir = platform.is_valid_snoop_filter_id(m->src_id());
    
    const bool is_data_from_dir_ack_zero =
      is_from_dir && (m->ack_count() == 0);
    
    const bool is_data_from_dir_ack_non_zero =
      is_from_dir && (m->ack_count() != 0);
    
    const bool is_data_from_owner = !is_from_dir;

    if (is_data_from_dir_ack_zero || is_data_from_owner) {

      switch (cache_line.state()) {
        case MosiAgentLineState::IS_D:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::S);
          a.set_transaction_done(true);
          break;
          
        case MosiAgentLineState::IM_AD:
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::M);
          a.set_transaction_done(true);
          break;
          
        default:
          a.set_error(true);
          break;
      }
      
    } else if (is_data_from_dir_ack_non_zero) {

      switch (cache_line.state()) {
        case MosiAgentLineState::IM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::IM_A);
          break;
          
        case MosiAgentLineState::SM_AD:
          a.append_command(CoherentAgentCommand::UpdateState);
          a.set_next_state(MosiAgentLineState::SM_A);
          break;
          
        default:
          a.set_error(true);
          break;
      }

    }
  }

  void handle__AckCount (
      const Message * m, const CacheLine & cache_line, CoherenceActions & a) const {

    switch (cache_line.state()) {
      case MosiAgentLineState::OM_AC:
        a.append_command(CoherentAgentCommand::SetAckCount);
        a.set_ack_count(m->ack_count());
        break;
      
      default:
        a.set_error(true);
        break;
    }
  }
  
  const CoherentAgentOptions opts_;
};

MosiCoherentAgentModel::MosiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MosiCoherentAgentModelImpl>(opts);
}

MosiCoherentAgentModel::~MosiCoherentAgentModel() {
}

void MosiCoherentAgentModel::init(CacheLine & l) const {
  impl_->init(l);
}

bool MosiCoherentAgentModel::is_stable(const CacheLine & l) const {
  return impl_->is_stable(l.state());
}

std::string MosiCoherentAgentModel::to_string(state_t s) const {
  return impl_->to_string(s);
}
  
//
CoherenceActions MosiCoherentAgentModel::get_actions(
    const Transaction * t, const CacheLine & cache_line) const {
  return impl_->get_actions(t, cache_line);
}

CoherenceActions MosiCoherentAgentModel::get_actions(
    const Message * m, const CacheLine & cache_line) const {
  return impl_->get_actions(m, cache_line);
}

struct MosiSnoopFilterModel::MosiSnoopFilterModelImpl {
  
  MosiSnoopFilterModelImpl(const SnoopFilterOptions & opts)
      : opts_(opts) 
  {}

  void init(DirectoryEntry & l) const {
    l.set_state(MosiDirectoryLineState::I);
  }

  bool is_stable(const DirectoryEntry & dir_entry) const {
    return MosiDirectoryLineState::to_string(dir_entry.state());
  }

  std::string to_string(const DirectoryEntry & dir_entry) const {
    std::stringstream ss;
    ss << MosiDirectoryLineState::to_string(dir_entry.state());
    return ss.str();
  }

  std::string to_string(state_t state) const {
    return MosiDirectoryLineState::to_string(state);
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
          
      case MessageType::PutO:
        handle__PutM(m, dir_entry, actions);
        break;
          
      default:
        actions.set_error(true);
    }
    return actions;
  }
 private:

  void handle__GetS(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MosiDirectoryLineState::S);
        break;

      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::SendDataToReq);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;

      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        break;

      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendFwdGetSToOwner);
        a.append_command(SnoopFilterCommand::AddReqToSharers);
        a.append_command(SnoopFilterCommand::UpdateState);
        a.set_next_state(MosiDirectoryLineState::O);
        break;

      default:
        a.set_error(true);
        break;
    }
  }
  
  void handle__GetM(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {

    const bool is_from_owner = false; // TODO

    if (is_from_owner) {
    
      switch (dir_entry.state()) {
        case MosiDirectoryLineState::O:
          a.append_command(SnoopFilterCommand::SendAckCountToReq);
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::O);
          break;

        default:
          a.set_error(true);
          break;
      }

    } else {
    
      switch (dir_entry.state()) {
        case MosiDirectoryLineState::I:
          a.append_command(SnoopFilterCommand::SendDataToReq);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;

        case MosiDirectoryLineState::S:
          a.append_command(SnoopFilterCommand::SendDataToReq);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;

        case MosiDirectoryLineState::O:
          a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
          a.append_command(SnoopFilterCommand::SendInvToSharers);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          a.append_command(SnoopFilterCommand::ClearSharers);
          a.append_command(SnoopFilterCommand::SendAckCountToReq);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::M);
          break;

        case MosiDirectoryLineState::M:
          a.append_command(SnoopFilterCommand::SendFwdGetMToOwner);
          a.append_command(SnoopFilterCommand::SetOwnerToReq);
          break;

        default:
          a.set_error(true);
          break;
      }

    }
  }
  
  void handle__PutS(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {

    const bool is_last = false; // TODO

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);
        break;

      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);        
        if (is_last) {
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;

      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);        
        break;

      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutSAckToReq);        
        break;

      default:
        a.set_error(true);
        break;
    }
  }
  
  void handle__PutM(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {

    const bool data_is_from_owner = false; // TODO

    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        break;

      case MosiDirectoryLineState::S:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        break;

      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::DelReqFromSharers);
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;

      case MosiDirectoryLineState::M:
        a.append_command(SnoopFilterCommand::SendPutMAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::I);
        }
        break;

      default:
        a.set_error(true);
        break;
    }
  }
  
  void handle__PutO(
      const Message * m, const DirectoryEntry & dir_entry, CoherenceActions & a) const {

    const bool data_is_from_owner = false; // TODO
    
    switch (dir_entry.state()) {
      case MosiDirectoryLineState::I:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        break;

      case MosiDirectoryLineState::S:
        if (!data_is_from_owner) {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        }
        break;

      case MosiDirectoryLineState::O:
        a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        if (data_is_from_owner) {
          a.append_command(SnoopFilterCommand::CpyDataToMemory);
          a.append_command(SnoopFilterCommand::DelOwner);
          a.append_command(SnoopFilterCommand::UpdateState);
          a.set_next_state(MosiDirectoryLineState::S);
        } else {
          a.append_command(SnoopFilterCommand::DelReqFromSharers);
        }
        break;

      case MosiDirectoryLineState::M:
        if (!data_is_from_owner)
          a.append_command(SnoopFilterCommand::SendPutOAckToReq);
        break;

      default:
        a.set_error(true);
        break;
    }
  }

  const SnoopFilterOptions opts_;
};

MosiSnoopFilterModel::MosiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
  impl_ = std::make_unique<MosiSnoopFilterModelImpl>(opts);
}

MosiSnoopFilterModel::~MosiSnoopFilterModel() {
}

void MosiSnoopFilterModel::init(DirectoryEntry & l) const {
  impl_->init(l);
}

bool MosiSnoopFilterModel::is_stable(const DirectoryEntry & l) const {
  return impl_->is_stable(l);
}

std::string MosiSnoopFilterModel::to_string(const DirectoryEntry & l) const {
  return impl_->to_string(l);
}

std::string MosiSnoopFilterModel::to_string(state_t state) const {
  return impl_->to_string(state);
}

CoherenceActions MosiSnoopFilterModel::get_actions(
    const Message * m, const DirectoryEntry & dir_entry) const {
  return impl_->get_actions(m, dir_entry);
}

} // namespace ccm
