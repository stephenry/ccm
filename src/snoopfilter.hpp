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

#ifndef __SRC_SNOOPFILTER_HPP__
#define __SRC_SNOOPFILTER_HPP__

#include "coherence.hpp"
#ifdef ENABLE_JSON
#  include <nlohmann/json.hpp>
#  include <memory>
#endif

namespace ccm {

class LoggerScope;

struct SnoopFilterOptions : CoherentAgentOptions {
#ifdef ENABLE_JSON
  static SnoopFilterOptions from_json(
      const Platform & platform, LoggerScope * l, nlohmann::json & j);
                                      
#endif

  SnoopFilterOptions(std::size_t id, const Platform & platform,
                     CacheOptions cache_options)
      : CoherentAgentOptions(id, platform, cache_options) {}
};

struct SnoopFilterCommandInvoker : CoherentActor {
  SnoopFilterCommandInvoker(const SnoopFilterOptions& opts);

  DirectoryEntry directory_entry(std::size_t addr) const;

  void visit_cache(CacheVisitor* cache_visitor) const override;
  void execute(Context& context, Cursor& cursor,
               const CoherenceActions& actions, const Message* msg,
               DirectoryEntry& d);

 protected:
  std::unique_ptr<SnoopFilterProtocol> cc_model_;
  std::unique_ptr<GenericCache<DirectoryEntry> > cache_;

 private:
  void execute_update_state(Context& context, Cursor& cursor, DirectoryEntry& d,
                            state_t state_next);
  void execute_set_owner_to_req(const Message* msg, Context& context,
                                Cursor& cursor, DirectoryEntry& d);
  void execute_send_data_to_req(const Message* msg, Context& context,
                                Cursor& cursor, DirectoryEntry& d,
                                const CoherenceActions& act);
  void execute_send_inv_to_sharers(const Message* msg, Context& context,
                                   Cursor& cursor, DirectoryEntry& d,
                                   const CoherenceActions& actions);
  void execute_clear_sharers(const Message* msg, Context& context,
                             Cursor& cursor, DirectoryEntry& d);
  void execute_add_req_to_sharers(const Message* msg, Context& context,
                                  Cursor& cursor, DirectoryEntry& d);
  void execute_del_req_from_sharers(const Message* msg, Context& context,
                                    Cursor& cursor, DirectoryEntry& d);
  void execute_del_owner(const Message* msg, Context& context, Cursor& cursor,
                         DirectoryEntry& d);
  void execute_add_owner_to_sharers(const Message* msg, Context& context,
                                    Cursor& cursor, DirectoryEntry& d);
  void execute_cpy_data_to_memory(const Message* msg, Context& context,
                                  Cursor& cursor, DirectoryEntry& d);
  void execute_send_puts_ack_to_req(const Message* msg, Context& context,
                                    Cursor& cursor, DirectoryEntry& d);
  void execute_send_putm_ack_to_req(const Message* msg, Context& context,
                                    Cursor& cursor, DirectoryEntry& d);
  void execute_send_fwd_gets_to_owner(const Message* msg, Context& context,
                                      Cursor& cursor, DirectoryEntry& d,
                                      const CoherenceActions& actions);
  void execute_send_pute_ack_to_req(const Message* msg, Context& context,
                                    Cursor& cursor);
  void execute_send_puto_ack_to_req(const Message* msg, Context& context,
                                    Cursor& cursor);
  void execute_send_ack_count_to_req(const Message* msg, Context& context,
                                     Cursor& cursor,
                                     const CoherenceActions& actions);
  void execute_send_fwd_getm_to_owner(const Message* msg, Context& context,
                                      Cursor& cursor, DirectoryEntry& d,
                                      const CoherenceActions& actions);

  MessageDirector msgd_;
  const SnoopFilterOptions opts_;
};

struct SnoopFilter : SnoopFilterCommandInvoker {
  SnoopFilter(const SnoopFilterOptions& opts);

  bool is_active() const override { return !qmgr_.empty(); }
  void apply(TimeStamped<Message*> ts) override;
  void eval(Context& context) override;

 private:
  void handle_msg(Context& context, Cursor& cursor, TimeStamped<Message*> ts);
  QueueManager qmgr_;
  const SnoopFilterOptions opts_;
};
#ifdef ENABLE_JSON

struct SnoopFilterBuilder {
  static std::unique_ptr<SnoopFilter> construct(
      const Platform & platform, LoggerScope * l, nlohmann::json j);
};
#endif

}  // namespace ccm

#endif
