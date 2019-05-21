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

#include <memory>
#include <optional>
#include "protocol.hpp"
#include "cache.hpp"
#include "message.hpp"
#include "sim.hpp"

namespace ccm {

class Transaction;
class Frontier;
class CoherentActor;
class Platform;
class CacheLine;
class DirectoryEntry;

using state_t = uint8_t;
using result_t = uint8_t;
using command_t = uint8_t;

struct CoherentAgentOptions : ActorOptions {
  CoherentAgentOptions(std::size_t id, Protocol protocol,
                       CacheOptions cache_options, Platform& platform)
      : ActorOptions(id, platform),
        protocol_(protocol),
        cache_options_(cache_options) {}
  Protocol protocol() const { return protocol_; }
  CacheOptions cache_options() const { return cache_options_; }

 private:
  Protocol protocol_;
  CacheOptions cache_options_;
};

}  // namespace ccm

#endif
