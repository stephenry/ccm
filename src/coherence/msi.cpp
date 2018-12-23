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

#define MSI_HANDLER_EMIT_CLASSES(__func)        \
  __func(Nop)                                   \
  __func(EmitGetS)                              \
  __func(EmitGetM)                              \
  __func(EmitInvAck)                            \
  __func(EmitDataReq)                           \
  __func(EmitDataDir)                           \
  __func(EmitFwdGetSToOwner)

enum class MsiHandlerEmitType {
#define __declare_enum(e) e,
  MSI_HANDLER_EMIT_CLASSES(__declare_enum)
#undef __declare_enum
};

const char * to_string(MsiHandlerEmitType t) {
  switch (t) {
#define __declare_to_string(__e)                \
    case MsiHandlerEmitType::__e: return #__e;
    MSI_HANDLER_EMIT_CLASSES(__declare_to_string)
#undef __declare_to_string
    default:
      return "<Invalid Line State>";
  }
}

struct MsiAgentHandlerAction : public CoherentAgentAction {
  std::vector<MsiHandlerEmitType> handler_emit;
  std::optional<MsiAgentLineState> state_update;
};

struct MsiCoherentAgentModel::MsiCoherentAgentModelImpl {
  MsiCoherentAgentModelImpl(const CoherentAgentOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}
  
  CoherentAgentAction apply(CoherentAgentContext & ctxt, CoherencyMessage * m) {
    CoherentAgentAction ret;

    if (message_requires_eviction(m)) {
      // TODO: require eviction mechanism
      ret.response = ResponseType::Stall;
    } else {
      // Dispatch to handler

      // TODO: need to look up the cache here (at the point ensured to
      // either hit in the cache, or space is available.
      //
      MsiAgentLineState l{MsiAgentLineState::I};

      MsiAgentHandlerAction h;
      switch (m->type()) {

        case MessageType::Load:
          h = handle__Load(l, static_cast<LoadCoherencyMessage *>(m));
          break;

        case MessageType::Store:
          h = handle__Store(l, static_cast<StoreCoherencyMessage *>(m));
          break;

        case MessageType::FwdGetS:
          h = handle__FwdGetS(l, static_cast<FwdGetSCoherencyMessage * >(m));
          break;

        case MessageType::FwdGetM:
          h = handle__FwdGetM(l, static_cast<FwdGetMCoherencyMessage * >(m));
          break;

        case MessageType::Inv:
          h = handle__Inv(l, static_cast<InvCoherencyMessage * >(m));
          break;

        case MessageType::PutS:
        case MessageType::PutM:
          h = handle__PutAck(l);
          break;

        case MessageType::Data:
          h = handle__Data(l);
          break;

        default:
            // TOOD: invalid message class.
            ;
      }

      // Commit condition: After the state update table has been
      // consulted, commit result only if sufficient credits are
      // available in the transaction-pool to emit all of the resultant
      // transaction to other agents/directory.
      //
      const bool do_commit = idpool_.available(h.handler_emit.size());
      if (do_commit)
        ret = construct_coherent_agent_action(h);
    }

    if (ret.message_consumed)
      m->release();
    
    return ret;
  }

  bool message_requires_eviction(CoherencyMessage * m) {
    bool ret = false;
    switch (m->type()) {
      case MessageType::Load:
      case MessageType::Store:
        ret = cache_.requires_eviction(m->addr());
        break;
      default:
        ret = false;
    }
    return ret;
  }

  MsiAgentHandlerAction handle__Load(MsiAgentLineState l, LoadCoherencyMessage * m) {
    MsiAgentHandlerAction a;
    switch (l) {
      case MsiAgentLineState::I:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitGetS);
        a.state_update = MsiAgentLineState::IS_D;
      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.response = ResponseType::Stall;
        break;

      case MsiAgentLineState::S:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::M:
        a.response = ResponseType::Hit;
        break;
    }
    return a;
  }
  
  MsiAgentHandlerAction handle__Store(MsiAgentLineState l, StoreCoherencyMessage * m) {
    MsiAgentHandlerAction a;
    switch (l) {
      case MsiAgentLineState::I:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitGetM);
        a.state_update = MsiAgentLineState::IM_AD;
      case MsiAgentLineState::IS_D:
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.response = ResponseType::Stall;
        break;
      case MsiAgentLineState::S:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitGetM);
        a.state_update = MsiAgentLineState::SM_AD;
        a.response = ResponseType::Stall;
        break;
      case MsiAgentLineState::M:
        a.response = ResponseType::Hit;
        break;
      default:
        // TOOD: invalid message received.
        ;
    }
    return a;
  }

  MsiAgentHandlerAction handle__FwdGetS(MsiAgentLineState l, FwdGetSCoherencyMessage * m) {
    MsiAgentHandlerAction a;
    switch (l) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.response = ResponseType::Stall;
        break;
      case MsiAgentLineState::M:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataReq);
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataDir);
        a.state_update = MsiAgentLineState::S;
        break;
      case MsiAgentLineState::MI_A:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataReq);
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataDir);
        a.state_update = MsiAgentLineState::SI_A;
        break;
      default:
        // TOOD: invalid message received.
        ;
    }
    return a;
  }

  MsiAgentHandlerAction handle__FwdGetM(MsiAgentLineState l, FwdGetMCoherencyMessage * m) {
    MsiAgentHandlerAction a;
    switch (l) {
      case MsiAgentLineState::IM_AD:
      case MsiAgentLineState::IM_A:
      case MsiAgentLineState::SM_AD:
      case MsiAgentLineState::SM_A:
        a.response = ResponseType::Stall;
        break;
      case MsiAgentLineState::M:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataReq);
        a.state_update = MsiAgentLineState::S;
        break;
      case MsiAgentLineState::MI_A:
        a.handler_emit.push_back(MsiHandlerEmitType::EmitDataReq);
        a.state_update = MsiAgentLineState::SI_A;
        break;
      default:
        // TOOD: invalid message received.
        ;
    }
    return a;
  }

  MsiAgentHandlerAction handle__Inv(MsiAgentLineState l, InvCoherencyMessage * m) {
    
    MsiAgentHandlerAction a;

    if (m->is_ack()) {
      // Line awaits invalidation acknowledgements from other agents
      // before upgrade to modified state.
      //
      // TODO: retain this on a line-by-line basis.
      const bool is_last_ack = false;

      if (is_last_ack) {
        switch (l) {
          case MsiAgentLineState::IM_A:
          case MsiAgentLineState::SM_A:
            a.state_update = MsiAgentLineState::M;
            break;
            
          default:
            ;
        }
      }

      // Inv-Ack are always sunk.
      a.message_consumed = true;
      
    } else {
      // Inbound invalidation request from the home directory. Update
      // line state accordingly.
      //
      switch (l) {
        case MsiAgentLineState::IS_D:
          a.response = ResponseType::Stall;
          break;
        case MsiAgentLineState::S:
          a.handler_emit.push_back(MsiHandlerEmitType::EmitInvAck);
          a.state_update = MsiAgentLineState::I;
          break;
        case MsiAgentLineState::SM_AD:
          a.handler_emit.push_back(MsiHandlerEmitType::EmitInvAck);
          a.state_update = MsiAgentLineState::IM_AD;
          break;
        case MsiAgentLineState::SI_A:
          a.handler_emit.push_back(MsiHandlerEmitType::EmitInvAck);
          a.state_update = MsiAgentLineState::II_A;
          break;
        default:
          // TOOD: invalid message received.
          ;
      }
    }
    return a;
  }

  MsiAgentHandlerAction handle__PutAck(MsiAgentLineState l) {
    MsiAgentHandlerAction a;
    switch (l) {
      case MsiAgentLineState::MI_A:
      case MsiAgentLineState::SI_A:
      case MsiAgentLineState::II_A:
        a.state_update = MsiAgentLineState::I;
      default:
        // TOOD: invalid message received.
        ;
    }
    return a;
  }

  MsiAgentHandlerAction handle__Data(MsiAgentLineState l) {
    MsiAgentHandlerAction a;

    // TODO: retain this on a line-by-line basis.

    bool is_from_dir = false;
    bool is_last_ack = false;

    if (is_last_ack) {

      // On the last 'Ack' (the last chuck of data received from
      // other agents in the system, the line state may advance
      // from the awaiting-ack state to the target stable state
      // (M or S).
      //
      switch (l) {
        case MsiAgentLineState::IS_D:
          a.state_update = MsiAgentLineState::S;
          break;
        case MsiAgentLineState::IM_AD:
          a.state_update = MsiAgentLineState::M;
          break;
        case MsiAgentLineState::SM_AD:
          a.state_update = MsiAgentLineState::M;
          break;
        default:
          // TODO: Invalid state
          ;
      }


    } else {

      // Data has been received, but we are currently awaiting
      // pending responses from other agents in the system.
      //
      switch (l) {
        case MsiAgentLineState::IM_AD:
          a.state_update = MsiAgentLineState::IM_A;
          break;
        case MsiAgentLineState::SM_AD:
          a.state_update = MsiAgentLineState::SM_A;
          break;
        default:
          // TODO: Invalid state
          ;
      }

    }
    
    return a;
  }

  CoherentAgentAction construct_coherent_agent_action(MsiAgentHandlerAction & h) {
    CoherentAgentAction ret;
    
    for (MsiHandlerEmitType emit : h.handler_emit) {
      switch (emit) {
        case MsiHandlerEmitType::EmitGetS:
          ret.msgs.push_back(create_gets());
          break;

        case MsiHandlerEmitType::EmitGetM:
          ret.msgs.push_back(create_getm());
          break;

        case MsiHandlerEmitType::EmitInvAck:
          ret.msgs.push_back(create_invack());
          break;

        case MsiHandlerEmitType::EmitDataReq:
          ret.msgs.push_back(create_datareq());
          break;

        case MsiHandlerEmitType::EmitDataDir:
          ret.msgs.push_back(create_datadir());
          break;

        default:
          // TODO: invalid emit request command
          ;
      }
    }
    ret.message_consumed = true;
    return ret;
  }

  GetSCoherencyMessage * create_gets() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
      
    GetSCoherencyMessageBuilder b = gets_.builder();
    //    b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  GetMCoherencyMessage * create_getm() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    GetMCoherencyMessageBuilder b = getm_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  InvCoherencyMessage * create_invack() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    InvCoherencyMessageBuilder b = inv_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    b.set_is_ack();
    return b.msg();
  }

  DataCoherencyMessage * create_datareq() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    DataCoherencyMessageBuilder b = data_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  DataCoherencyMessage * create_datadir() {
    std::size_t tid;
    CCM_ASSERT(idpool_.get_id(tid));
    
    DataCoherencyMessageBuilder b = data_.builder();
    //  b.set_addr(m->addr());
    b.set_tid(tid);
    return b.msg();
  }

  CacheModel<MsiAgentLineState> cache_;
  CoherentAgentOptions opts_;
  GetSCoherencyMessageDirector gets_;
  GetMCoherencyMessageDirector getm_;
  InvCoherencyMessageDirector inv_;
  DataCoherencyMessageDirector data_;
  IdPool idpool_;
};

MsiCoherentAgentModel::MsiCoherentAgentModel(const CoherentAgentOptions & opts)
    : CoherentAgentModel(opts) {
  impl_ = std::make_unique<MsiCoherentAgentModelImpl>(opts);
}

MsiCoherentAgentModel::~MsiCoherentAgentModel() {};

CoherentAgentAction MsiCoherentAgentModel::apply(CoherentAgentContext & ctxt,
                                                 CoherencyMessage * m) {
  return impl_->apply(ctxt, m);
}

#define DIRECTORY_STATES(__func)                \
  __func(I)                                     \
  __func(S)                                     \
  __func(M)                                     \
  __func(S_D)

enum class MsiDirectoryLineState {
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

#define SNOOP_FILTER_ACTION_STATES(__func)      \
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


enum class MsiSnoopFilterHandlerAction {
#define __declare_state(__state)                \
  __state,
  SNOOP_FILTER_ACTION_STATES(__declare_state)
#undef __declare_state
};

struct MsiDirectoryHandlerAction : SnoopFilterAction {

  bool has_state_update() const { return next_state.has_value(); }
  MsiDirectoryLineState get_next_state() const { return next_state.value(); }
  void set_next_state(MsiDirectoryLineState state) { next_state = state; }

  std::size_t message_count() const { return msgs_n_; }
  
  void add_action(const MsiSnoopFilterHandlerAction & a) {
    actions.push_back(a);
  }
  void set_stall(bool stall = true) { stall = stall; }
  
  bool stall;
  std::optional<MsiDirectoryLineState> next_state;
  std::vector<MsiSnoopFilterHandlerAction> actions;
  std::size_t msgs_n_{0};
};

struct MsiDirectoryHandlerContext {

  //
  MsiDirectoryHandlerContext(MsiDirectoryLineState state)
      : state_(state)
  {}

  //
  MsiDirectoryLineState state() const { return state_; }
  bool is_last() const { return false; }
  std::size_t req_id() const { return 0; }
  bool is_data_from_owner() const { return false; }

 private:
  MsiDirectoryLineState state_;
};

struct DirEntry {

  DirEntry() : state(MsiDirectoryLineState::I) {}

  //
  MsiDirectoryLineState get_state() const { return state; }
  void set_state(MsiDirectoryLineState state) { state = state; }
    
  //
  std::size_t get_owner() const { return owner.value(); }
  void set_owner(std::size_t owner) { owner = owner; }
  void clear_owner() { owner.reset(); }

  //
  void add_sharer(std::size_t id) { sharers.push_back(id); }
  void remove_sharer(std::size_t id) {
    sharers.erase(std::find(sharers.begin(), sharers.end(), id), sharers.end());
  }
  void remove_all_sharers() { sharers.clear(); }

  // Current state of the line
  MsiDirectoryLineState state;

  // Current set of sharing agents
  std::vector<std::size_t> sharers;

  // Current owner agent
  std::optional<std::size_t> owner;
};

struct MsiSnoopFilterModel::MsiSnoopFilterModelImpl {
  
  MsiSnoopFilterModelImpl(const SnoopFilterOptions & opts)
      : opts_(opts), cache_(opts.cache_options)
  {}
  
  SnoopFilterAction apply(CoherentAgentContext & ctxt, CoherencyMessage * m) {
    SnoopFilterAction ret;
    if (message_requires_recall(m)) {

    } else {

      CCM_ASSERT(cache_.is_hit(m->addr()));

      bool advances = false;
      const DirEntry & dir_entry = cache_.entry(m->addr());
      MsiDirectoryHandlerAction actions;
      
      switch (m->type()) {
        case MessageType::GetS:
          advances = handle__GetS(
              static_cast<GetSCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::GetM:
          advances = handle__GetM(
              static_cast<GetMCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::PutS:
          advances = handle__PutS(
              static_cast<PutSCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::PutM:
          advances = handle__PutM(
              static_cast<PutMCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        case MessageType::Data:
          advances = handle__Data(
              static_cast<DataCoherencyMessage *>(m), dir_entry, actions);
          break;
          
        default:
          // TODO: Unknown message type
          advances = false;
      }

      if (!advances) {
        ret.set_result(SnoopFilterActionResult::BlockedOnProtocol);
        return ret;
      }

      if (!idpool_.available(actions.message_count())) {
        ret.set_result(SnoopFilterActionResult::TagsExhausted);
        return ret;
      }

      commit_actions(m, actions, dir_entry, ret);
      ret.set_result(SnoopFilterActionResult::Advances);
    }
    return ret;
  }

  bool handle__GetS(GetSCoherencyMessage * m,
                    const DirEntry & dir_entry,
                    MsiDirectoryHandlerAction & act) {
    bool advances = true;
    switch (dir_entry.get_state()) {
      case MsiDirectoryLineState::I:
        act.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        act.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        act.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S:
        act.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        act.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        break;
      case MsiDirectoryLineState::M:
        act.add_action(MsiSnoopFilterHandlerAction::SendFwdGetSToOwner);
        act.add_action(MsiSnoopFilterHandlerAction::AddReqToSharers);
        act.add_action(MsiSnoopFilterHandlerAction::AddOwnerToSharers);
        act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        act.set_next_state(MsiDirectoryLineState::S);
        break;
      case MsiDirectoryLineState::S_D:
        advances = false;
        break;
      default:
        // NOP
        ;
    }
    return true;
  }

  bool handle__GetM(GetMCoherencyMessage * m,
                    const DirEntry & dir_entry,
                    MsiDirectoryHandlerAction & act) {
    bool advances = true;
    switch (dir_entry.get_state()) {
      case MsiDirectoryLineState::I:
        act.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        act.add_action(MsiSnoopFilterHandlerAction::SetOwnerToReq);
        act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        act.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::S:
        act.add_action(MsiSnoopFilterHandlerAction::SendDataToReq);
        act.add_action(MsiSnoopFilterHandlerAction::SendInvToSharers);
        act.add_action(MsiSnoopFilterHandlerAction::ClearSharers);
        act.add_action(MsiSnoopFilterHandlerAction::SetOwnerToReq);
        act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        act.set_next_state(MsiDirectoryLineState::M);
        break;
      case MsiDirectoryLineState::M:
        break;
      case MsiDirectoryLineState::S_D:
        advances = false;
        break;
      default:
        // NOP
        ;
    }
    return advances;
  }

  bool handle__PutS(PutSCoherencyMessage * m,
                    const DirEntry & dir_entry,
                    MsiDirectoryHandlerAction & act) {
    const bool is_last = false; // TODO
    bool advances = true;
    switch (dir_entry.get_state()) {
      case MsiDirectoryLineState::I:
        act.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S:
        act.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        act.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        if (is_last) {
          act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
          act.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::M:
        act.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        act.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      case MsiDirectoryLineState::S_D:
        act.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
        act.add_action(MsiSnoopFilterHandlerAction::SendPutSAckToReq);
        break;
      default:
        // NOP
        ;
    }
    return advances;
  }

  bool handle__PutM(PutMCoherencyMessage * m,
                    const DirEntry & dir_entry, 
                    MsiDirectoryHandlerAction & act) {
    bool advances = true;

    const bool is_data_from_owner = (m->mid() == dir_entry.owner.value());
    switch (dir_entry.get_state()) {
      case MsiDirectoryLineState::I:
        if (!is_data_from_owner) {
          act.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::S:
        if (!is_data_from_owner) {
          act.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
          act.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      case MsiDirectoryLineState::M:
        act.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        if (is_data_from_owner) {
          act.add_action(MsiSnoopFilterHandlerAction::DelOwner);
          act.add_action(MsiSnoopFilterHandlerAction::CpyDataToMemory);
          act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
          act.set_next_state(MsiDirectoryLineState::I);
        }
        break;
      case MsiDirectoryLineState::S_D:
        if (!is_data_from_owner) {
          act.add_action(MsiSnoopFilterHandlerAction::DelReqFromSharers);
          act.add_action(MsiSnoopFilterHandlerAction::SendPutMAckToReq);
        }
        break;
      default:
        // NOP
        ;
    }
    return advances;
  }

  bool handle__Data(DataCoherencyMessage * m,
                    const DirEntry & dir_entry,
                    MsiDirectoryHandlerAction & act) {
    bool advances = true;
    switch (dir_entry.get_state()) {
      case MsiDirectoryLineState::S_D:
        act.add_action(MsiSnoopFilterHandlerAction::CpyDataToMemory);
        act.add_action(MsiSnoopFilterHandlerAction::UpdateState);
        act.set_next_state(MsiDirectoryLineState::S);
        break;
      default:
        // NOP
        ;
    }
    return advances;
  }

  bool message_requires_recall(CoherencyMessage * m) {
    if (opts_.is_null_filter)
      return false;

    // TODO: Directory eviction code.
    
    return false;
  }

  void commit_actions(const CoherencyMessage * m,
                      const MsiDirectoryHandlerAction & a,
                      DirEntry & dir_entry,
                      SnoopFilterAction & ret) {
    const std::size_t mid = m->mid();
    for (MsiSnoopFilterHandlerAction action : a.actions) {
      
      switch (action) {
        case MsiSnoopFilterHandlerAction::UpdateState:
          CCM_ASSERT(a.has_state_update());
          
          dir_entry.set_state(a.get_next_state());
          break;
          
        case MsiSnoopFilterHandlerAction::SetOwnerToReq:
          dir_entry.set_owner(mid);
          break;
          
        case MsiSnoopFilterHandlerAction::SendDataToReq:
          ret.add_msg(construct_Data(mid));
          break;
          
        case MsiSnoopFilterHandlerAction::SendInvToSharers:
          for (std::size_t sharer_id : dir_entry.sharers)
            ret.add_msg(construct_Inv(sharer_id));
          break;
          
        case MsiSnoopFilterHandlerAction::ClearSharers:
          dir_entry.remove_all_sharers();
          break;
          
        case MsiSnoopFilterHandlerAction::AddReqToSharers:
          dir_entry.add_sharer(mid);
          break;
          
        case MsiSnoopFilterHandlerAction::DelReqFromSharers:
          dir_entry.remove_sharer(mid);
          break;
          
        case MsiSnoopFilterHandlerAction::DelOwner:
          dir_entry.clear_owner();
          break;
          
        case MsiSnoopFilterHandlerAction::AddOwnerToSharers:
          dir_entry.add_sharer(dir_entry.get_owner());
          break;
          
        case MsiSnoopFilterHandlerAction::CpyDataToMemory:
          // TODO
          break;
          
        case MsiSnoopFilterHandlerAction::SendPutSAckToReq:
          ret.add_msg(construct_PutSAck(mid));
          break;
          
        case MsiSnoopFilterHandlerAction::SendPutMAckToReq:
          ret.add_msg(construct_PutMAck(mid));
          break;
          
        case MsiSnoopFilterHandlerAction::SendFwdGetSToOwner:
          ret.add_msg(construct_FwdGetS(dir_entry.get_owner()));
          break;
      }
    }
  }

  FwdGetSCoherencyMessage * construct_FwdGetS(std::size_t dest_id) {
    FwdGetSCoherencyMessageBuilder b = fwdgets_.builder();
    return b.msg();
  }

  FwdGetMCoherencyMessage * construct_FwdGetM(std::size_t dest_id) {
    FwdGetMCoherencyMessageBuilder b = fwdgetm_.builder();
    return b.msg();
  }

  PutSCoherencyMessage * construct_PutSAck(std::size_t dest_id) {
    PutSCoherencyMessageBuilder b = puts_.builder();
    return b.msg();
  }

  PutMCoherencyMessage * construct_PutMAck(std::size_t dest_id) {
    PutMCoherencyMessageBuilder b = putm_.builder();
    return b.msg();
  }

  DataCoherencyMessage * construct_Data(std::size_t dest_id) {
    DataCoherencyMessageBuilder b = data_.builder();
    return b.msg();
  }

  InvCoherencyMessage * construct_Inv(std::size_t dest_id) {
    InvCoherencyMessageBuilder b = inv_.builder();
    return b.msg();
  }

  bool transaction_must_hit(MessageType t) {
    return true;
  }
  
  SnoopFilterOptions opts_;
  FwdGetSCoherencyMessageDirector fwdgets_;
  FwdGetMCoherencyMessageDirector fwdgetm_;
  PutSCoherencyMessageDirector puts_;
  PutMCoherencyMessageDirector putm_;
  InvCoherencyMessageDirector inv_;
  DataCoherencyMessageDirector data_;
  IdPool idpool_;
  CacheModel<DirEntry> cache_;
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
      : MsiSnoopFilterModelImpl(opts), cache_(opts.cache_options)
  {}
};

MsiSnoopFilterModel::MsiSnoopFilterModel(const SnoopFilterOptions & opts)
    : SnoopFilterModel(opts) {
  if (opts.is_null_filter) {
    // TODO: not presently supported by the protocol
    impl_ = std::make_unique<MsiSnoopFilterModelNullFilterImpl>(opts);
  } else {
    impl_ = std::make_unique<MsiSnoopFilterModelDirectoryImpl>(opts);
  }
}

MsiSnoopFilterModel::~MsiSnoopFilterModel() {}

SnoopFilterAction MsiSnoopFilterModel::apply(CoherentAgentContext & ctxt,
                                             CoherencyMessage * m) {
  return impl_->apply(ctxt, m);
}

} // namespace ccm
