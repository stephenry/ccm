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
#include "message.hpp"
#include "transaction.hpp"
#include "actors.hpp"
#include "common.hpp"
#include <algorithm>

namespace ccm {

#define LINE_STATES(__func)                     \
  __func(I)                                     \
  __func(IS_D)                                  \
  __func(IM_AD)                                 \
  __func(IM_A)                                  \
  __func(S)                                     \
  __func(SM_AD)                                 \
  __func(SM_A)                                  \
  __func(M)                                     \
  __func(MI_A)                                  \
  __func(SI_A)                                  \
  __func(II_A)

enum class MsiAgentLineState : uint8_t {
#define __declare_state(__state)                \
  __state,
  LINE_STATES(__declare_state)
#undef __declare_state
};

const char * to_string(MsiAgentLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiAgentLineState::__e: return #__e;
    LINE_STATES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
}

struct MsiCoherentAgentModel::MsiCoherentAgentModelImpl {
  struct LineEntry {
    LineEntry() : state_(MsiAgentLineState::I) {}
    
    MsiAgentLineState state() const { return state_; }
    void set_state(MsiAgentLineState state) { state_ = state; }
   private:
    MsiAgentLineState state_;
  };
  
  MsiCoherentAgentModelImpl(const AgentOptions & opts)
      : opts_(opts), cache_(opts.cache_options())
  {}

  CoherentActorActions get_actions(const Transaction * t) {
    CoherentActorActions actions;
    LineEntry line_entry;
    switch (t->type()) {
      case TransactionType::Load:
        handle__Load(t, line_entry, actions);
        break;
        
      case TransactionType::Store:
        handle__Store(t, line_entry, actions);
        break;
    }
    return actions;
  }
  
  CoherentActorActions get_actions(const Message * m) {
    CoherentActorActions actions;
    LineEntry line_entry;
    switch (m->type()) {
      case MessageType::FwdGetS:
        handle__FwdGetS(m, line_entry, actions);
        break;

      case MessageType::FwdGetM:
        handle__FwdGetM(m, line_entry, actions);
        break;

      case MessageType::Inv:
        handle__Inv(m, line_entry, actions);
        break;

      case MessageType::PutS:
      case MessageType::PutM:
        handle__PutAck(m, line_entry, actions);
        break;

      case MessageType::Data:
        handle__Data(m, line_entry, actions);
        break;

      default:
        actions.set_error();
    }
    return actions;
  }

  bool message_requires_eviction(const Message * m) {
    return false;
  }

  void handle__Load(const Transaction * t, const LineEntry & line_entry, CoherentActorActions & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(CoherentAgentCommand::EmitGetS);
        a.set_next_state(MsiAgentLineState::IS_D);
        [[fallthrough]];
        
      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_stall();
        break;

      case MsiAgentLineState::S:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::M:
        //        a.response = ResponseType::Hit;
        break;
        
      default:
        a.set_error();
    }
  }
  
  void handle__Store(const Transaction * t, const LineEntry & line_entry, CoherentActorActions & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::I:
        a.add_action(CoherentAgentCommand::EmitGetM);
        a.set_next_state(MsiAgentLineState::IM_AD);
        [[fallthrough]];

      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::S:
        a.add_action(CoherentAgentCommand::EmitGetM);
        a.set_next_state(MsiAgentLineState::SM_AD);
        a.set_stall();
        break;
                     
      case MsiAgentLineState::M:
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__FwdGetS(const Message * m, const LineEntry & line_entry, CoherentActorActions & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::M:
        a.add_action(CoherentAgentCommand::EmitDataToReq);
        a.add_action(CoherentAgentCommand::EmitDataToDir);
        a.set_next_state(MsiAgentLineState::S);
        break;
        
      case MsiAgentLineState::MI_A:
        a.add_action(CoherentAgentCommand::EmitDataToReq);
        a.add_action(CoherentAgentCommand::EmitDataToDir);
        a.set_next_state(MsiAgentLineState::SI_A);
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__FwdGetM(const Message * m, const LineEntry & line_entry, CoherentActorActions & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.set_stall();
        break;
        
      case MsiAgentLineState::M:
        a.add_action(CoherentAgentCommand::EmitDataToReq);
        a.set_next_state(MsiAgentLineState::S);
        break;
        
      case MsiAgentLineState::MI_A:
        a.add_action(CoherentAgentCommand::EmitDataToReq);
        a.set_next_state(MsiAgentLineState::SI_A);
        break;
        
      default:
        a.set_error();
    }
  }

  void handle__Inv(const Message *m, const LineEntry & line_entry, CoherentActorActions & a) {
    if (m->is_ack()) {
      // Line awaits invalidation acknowledgements from other agents
      // before upgrade to modified state.
      //
      // TODO: retain this on a line-by-line basis.
      const bool is_last_ack = false;

      if (is_last_ack) {
        switch (line_entry.state()) {
          case MsiAgentLineState::IM_A:
          case MsiAgentLineState::SM_A:
            a.set_next_state(MsiAgentLineState::M);
            break;
            
          default:
            a.set_error();
        }
      }
      
    } else {
      // Inbound invalidation request from the home directory. Update
      // line state accordingly.
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IS_D:
          a.set_stall();
          break;
          
        case MsiAgentLineState::S:
          a.add_action(CoherentAgentCommand::EmitInvAck);
          a.set_next_state(MsiAgentLineState::I);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.add_action(CoherentAgentCommand::EmitInvAck);
          a.set_next_state(MsiAgentLineState::IM_AD);
          break;
          
        case MsiAgentLineState::SI_A:
          a.add_action(CoherentAgentCommand::EmitInvAck);
          a.set_next_state(MsiAgentLineState::II_A);
          break;
          
        default:
          a.set_error();
      }
    }
  }

  void handle__PutAck(const Message * m, const LineEntry & line_entry, CoherentActorActions & a) {
    switch (line_entry.state()) {
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.set_next_state(MsiAgentLineState::I);
        break;
      default:
        a.set_error();
    }
  }

  void handle__Data(const Message * m, const LineEntry & line_entry, CoherentActorActions & a) {

    // TODO: retain this on a line-by-line basis.

    bool is_from_dir = false;
    bool is_last_ack = false;

    if (is_last_ack) {

      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).n
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IS_D:
          a.set_next_state(MsiAgentLineState::S);
          break;
          
        case MsiAgentLineState::IM_AD:
          a.set_next_state(MsiAgentLineState::M);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.set_next_state(MsiAgentLineState::M);
          break;
          
        default:
          a.set_error();
      }
    } else {

      // Data has been received, but we are currently awaiting
      // pending responses from other agents in the system.
      //
      switch (line_entry.state()) {
        case MsiAgentLineState::IM_AD:
          a.set_next_state(MsiAgentLineState::IM_A);
          break;
          
        case MsiAgentLineState::SM_AD:
          a.set_next_state(MsiAgentLineState::SM_A);
          break;
          
        default:
          a.set_error();
      }
    }
  }

  CacheModel<MsiAgentLineState> cache_;
  AgentOptions opts_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const AgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

CoherentActorActions MsiCoherentAgentModel::get_actions(const Transaction * t) {
  return impl_->get_actions(t);
}

CoherentActorActions MsiCoherentAgentModel::get_actions(const Message * m) {
  return impl_->get_actions(m);
}

#define DIRECTORY_STATES(__func)                \
  __func(I)                                     \
  __func(S)                                     \
  __func(M)                                     \
  __func(S_D)

enum class MsiDirectoryLineState : uint8_t {
#define __declare_state(__state)                \
  __state,
  DIRECTORY_STATES(__declare_state)
#undef __declare_state
};

const char * to_string(MsiDirectoryLineState s) {
  switch (s) {
#define __declare_to_string(__e)                \
    case MsiDirectoryLineState::__e: return #__e;
    DIRECTORY_STATES(__declare_to_string)
#undef __declare_to_string
  }
  return "<Invalid Directory State>";
}

struct MsiSnoopFilterModel::MsiSnoopFilterModelImpl {
  
  MsiSnoopFilterModelImpl(const SnoopFilterOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}
  
  CoherentActorActions get_actions(const Message * m) {
    CoherentActorActions actions;
    if (message_requires_recall(m)) {

    } else {

      CCM_ASSERT(cache_.is_hit(m->addr()));

      DirectoryEntry & dir_entry = cache_.entry(m->addr());

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
          actions.set_error();
      }
      CCM_ASSERT(!actions.error());

      // if (!actions.stall()) {
      //   ret.set_status(CoherentActorResultStatus::BlockedOnProtocol);
      //   return ret;
      // }

      // ret.set_status(CoherentActorResultStatus::Advances);
    }
    return actions;
  }
 private:
  
  void handle__GetS(const Message * m, const DirectoryEntry & dir_entry, CoherentActorActions & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(SnoopFilterCommand::SendDataToReq);
        a.add_action(SnoopFilterCommand::AddReqToSharers);
        a.add_action(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(SnoopFilterCommand::SendDataToReq);
        a.add_action(SnoopFilterCommand::AddReqToSharers);
        break;
      case MsiDirectoryLineState::M:
        a.add_action(SnoopFilterCommand::SendFwdGetSToOwner);
        a.add_action(SnoopFilterCommand::AddReqToSharers);
        a.add_action(SnoopFilterCommand::AddOwnerToSharers);
        a.add_action(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S_D:
        a.set_stall();
        break;
      default:
        a.set_error();
    }
  }

  void handle__GetM(const Message * m, const DirectoryEntry & dir_entry, CoherentActorActions & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(SnoopFilterCommand::SendDataToReq);
        a.add_action(SnoopFilterCommand::SetOwnerToReq);
        a.add_action(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(SnoopFilterCommand::SendDataToReq);
        a.add_action(SnoopFilterCommand::SendInvToSharers);
        a.add_action(SnoopFilterCommand::ClearSharers);
        a.add_action(SnoopFilterCommand::SetOwnerToReq);
        a.add_action(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::M:
        break;
      case MsiDirectoryLineState::S_D:
        a.set_stall();
        break;
      default:
        a.set_error();
    }
  }

  void handle__PutS(const Message * m, const DirectoryEntry & dir_entry, CoherentActorActions & a) const {
    const bool is_last = false; // TODO
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        a.add_action(SnoopFilterCommand::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S:
        a.add_action(SnoopFilterCommand::DelReqFromSharers);
        a.add_action(SnoopFilterCommand::SendPutSAckToReq);
        if (is_last) {
          a.add_action(SnoopFilterCommand::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::M:
        a.add_action(SnoopFilterCommand::DelReqFromSharers);
        a.add_action(SnoopFilterCommand::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S_D:
        a.add_action(SnoopFilterCommand::DelReqFromSharers);
        a.add_action(SnoopFilterCommand::SendPutSAckToReq);
        break;
      default:
        a.set_error();
    }
  }

  void handle__PutM(const Message * m, const DirectoryEntry & dir_entry, CoherentActorActions & a) const {
    const bool is_data_from_owner = false;
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::I:
        if (!is_data_from_owner) {
          a.add_action(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::S:
        if (!is_data_from_owner) {
          a.add_action(SnoopFilterCommand::DelReqFromSharers);
          a.add_action(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::M:
        a.add_action(SnoopFilterCommand::SendPutMAckToReq);
        if (is_data_from_owner) {
          a.add_action(SnoopFilterCommand::DelOwner);
          a.add_action(SnoopFilterCommand::CpyDataToMemory);
          a.add_action(SnoopFilterCommand::UpdateState);
          a.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::S_D:
        if (!is_data_from_owner) {
          a.add_action(SnoopFilterCommand::DelReqFromSharers);
          a.add_action(SnoopFilterCommand::SendPutMAckToReq);
        }
        break;
      default:
        a.set_error();
    }
  }

  void handle__Data(const Message * m, const DirectoryEntry & dir_entry, CoherentActorActions & a) const {
    switch (dir_entry.state()) {
      case MsiDirectoryLineState::S_D:
        a.add_action(SnoopFilterCommand::CpyDataToMemory);
        a.add_action(SnoopFilterCommand::UpdateState);
        a.set_next_state(MsiDirectoryLineState::S);
        break;
      default:
        a.set_error();
    }
  }

  bool message_requires_recall(const Message * m) {
    return false;
  }

  bool transaction_must_hit(MessageType t) {
    return true;
  }
  
  SnoopFilterOptions opts_;
  CacheModel<DirectoryEntry> cache_;
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelNullFilterImpl
    : MsiSnoopFilterModel::MsiSnoopFilterModelImpl {

  MsiSnoopFilterModelNullFilterImpl(const SnoopFilterOptions & opts)
      : MsiSnoopFilterModelImpl(opts)
  {}
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelDirectoryImpl
    : MsiSnoopFilterModel::MsiSnoopFilterModelImpl {

  MsiSnoopFilterModelDirectoryImpl(const SnoopFilterOptions & opts)
      : MsiSnoopFilterModelImpl(opts)
  {}
};

MsiSnoopFilterModel::MsiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
  impl_ = std::make_unique<MsiSnoopFilterModelDirectoryImpl>(opts);
}

MsiSnoopFilterModel::~MsiSnoopFilterModel() {}

CoherentActorActions MsiSnoopFilterModel::get_actions(const Message * m) {
  return impl_->get_actions(m);
}

} // namespace ccm
