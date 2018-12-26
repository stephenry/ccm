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

#ifndef __SRC_COHERENCE_HPP__
#define __SRC_COHERENCE_HPP__

#include "cache_model.hpp"
#include <memory>
#include <optional>

namespace ccm {

class Message;
class Transaction;

enum class Protocol {
  MSI,
  MESI,
  MOSI
};

struct CoherentAgentOptions {
  CoherentAgentOptions(std::size_t id)
      : id_(id)
  {}

  std::size_t id() const { return id_; }
  
  CacheOptions cache_options;

  std::size_t id_;
  
  std::size_t max_in_flight_n{16};
};

#define COHERENT_ACTOR_RESULT(__func)      \
  __func(Advances)                              \
  __func(BlockedOnProtocol)                     \
  __func(TagsExhausted)

enum class CoherentActorResultStatus {
#define __declare_enum(e) e,
  COHERENT_ACTOR_RESULT(__declare_enum)
#undef __declare_enum
};

struct CoherentActorResult {
  CoherentActorResult()
  {}

  CoherentActorResultStatus status() const { return status_; }

  void set_status(CoherentActorResultStatus status) { status_ = status; }
  void add_msg(Message * m) { msgs_.push_back(m); }

  std::vector<Message *> msgs() const { return msgs_; }

 private:
  CoherentActorResultStatus status_;
  std::vector<Message *> msgs_;
};

class CoherentActorBase {
 public:
  virtual Protocol protocol() const = 0;
  virtual CoherentActorResult apply(const Message * m) = 0;
  virtual ~CoherentActorBase() {}
};

class CoherentAgentModel : public CoherentActorBase {
 public:
  CoherentAgentModel(const CoherentAgentOptions & opts);
  virtual CoherentActorResult apply(const Transaction * t) = 0;
};

std::unique_ptr<CoherentAgentModel> coherent_agent_factory(
    Protocol protocol, 
    const CoherentAgentOptions & opts);

struct SnoopFilterOptions {
  SnoopFilterOptions(std::size_t id)
      : id_(id)
  {}

  std::size_t id() const { return id_; }
  
  CacheOptions cache_options;

  std::size_t id_;

  std::size_t num_agents{1};
};

class SnoopFilterModel : public CoherentActorBase {
 public:
  SnoopFilterModel(const SnoopFilterOptions & opts);
};

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    Protocol protocol, 
    const SnoopFilterOptions & opts);

} // namespace ccm

#endif
