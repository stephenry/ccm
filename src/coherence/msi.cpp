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
  __func(EmitDataDir)

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

struct MsiHandlerAction : public CoherentAgentAction {
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

      MsiHandlerAction h;
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

  MsiHandlerAction handle__Load(MsiAgentLineState l, LoadCoherencyMessage * m) {
    MsiHandlerAction a;
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
  
  MsiHandlerAction handle__Store(MsiAgentLineState l, StoreCoherencyMessage * m) {
    MsiHandlerAction a;
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

  MsiHandlerAction handle__FwdGetS(MsiAgentLineState l, FwdGetSCoherencyMessage * m) {
    MsiHandlerAction a;
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

  MsiHandlerAction handle__FwdGetM(MsiAgentLineState l, FwdGetMCoherencyMessage * m) {
    MsiHandlerAction a;
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

  MsiHandlerAction handle__Inv(MsiAgentLineState l, InvCoherencyMessage * m) {
    
    MsiHandlerAction a;

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
            a.state_update = MsiAgentLintState::M;
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

  MsiHandlerAction handle__PutAck(MsiAgentLineState l) {
    MsiHandlerAction a;
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

  MsiHandlerAction handle__Data(MsiAgentLineState l) {
    MsiHandlerAction a;

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
        case MsiHandlerEmitType::IS_D:
          a.state_update = MsiAgentLineState::S;
          break;
        case MsiHandlerEmitType::IM_AD:
          a.state_update = MsiAgentLineState::M;
          break;
        case MsiHandlerEmitType::SM_AD:
          a.state_update = MsiAgentLineState::M;
          break;
      }


    } else {

      // Data has been received, but we are currenlt awaiting
      // pending respons from other agents in the system.
      //
      switch (l) {
        case MsiHandlerEmitType::IM_AD:
          a.state_update = MsiAgentLineState::IM_A;
          break;
        case MsiHandlerEmitType::SM_AD:
          a.state_update = MsiAgentLineState::SM_A;
          break;
      }

    }
    
    return a;
  }

  CoherentAgentAction construct_coherent_agent_action(MsiHandlerAction & h) {
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

struct MsiDirectoryModel::MsiDirectoryModelImpl {
  MsiDirectoryModelImpl(const DirectoryOptions & opts)
      : opts_(opts)
  {}
  
  DirectoryAction apply(CoherentAgentContext & ctxt, CoherencyMessage * m) {
    return {};
  }
  
  DirectoryOptions opts_;
  FwdGetSCoherencyMessageDirector fwdgets_;
  FwdGetMCoherencyMessageDirector fwdgetm_;
};

MsiDirectoryModel::MsiDirectoryModel(const DirectoryOptions & opts)
    : DirectoryModel(opts) {
}

MsiDirectoryModel::~MsiDirectoryModel() {}

DirectoryAction MsiDirectoryModel::apply(CoherentAgentContext & ctxt,
                                         CoherencyMessage * m) {
  return impl_->apply(ctxt, m);
}

} // namespace ccm
