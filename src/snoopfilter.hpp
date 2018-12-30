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

namespace ccm {

struct SnoopFilterOptions : ActorOptions {
  SnoopFilterOptions(std::size_t id, Protocol protocol, CacheOptions cache_options)
      : ActorOptions(id), protocol_(protocol), cache_options_(cache_options)
  {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }
 private:
  Protocol protocol_;
  CacheOptions cache_options_;
};

struct SnoopFilter : CoherentActor {
  SnoopFilter(const SnoopFilterOptions & opts);
  
  bool is_active() const override { return !pending_messages_.empty(); }
  void apply(std::size_t t, const Message * m) override;
  bool eval(Frontier & f) override;
 private:
  Heap<TimeStamped<const Message *> > pending_messages_;
  std::unique_ptr<SnoopFilterModel> cc_model_;
  const SnoopFilterOptions & opts_;
};

std::unique_ptr<SnoopFilterModel> snoop_filter_factory(
    const SnoopFilterOptions & opts);

} // namespace ccm

#endif

