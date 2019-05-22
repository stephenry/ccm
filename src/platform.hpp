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

#ifndef __SRC_PLATFORM_HPP__
#define __SRC_PLATFORM_HPP__

#include <map>
#include <memory>
#include <set>
#include "common.hpp"
#ifdef ENABLE_JSON
#  include <nlohmann/json.hpp>
#endif
#include "types.hpp"
#include "protocol.hpp"

namespace ccm {

struct AddressRegion {
#ifdef ENABLE_JSON
  static std::shared_ptr<AddressRegion> from_json(nlohmann::json j);
#endif
  virtual bool is_valid(addr_t addr) const = 0;
};

struct DefaultAddressRegion : AddressRegion {
  bool is_valid(addr_t addr) const override { return true; }
};

struct ContiguousAddressRegion : AddressRegion {
#ifdef ENABLE_JSON
  static std::shared_ptr<AddressRegion> from_json(nlohmann::json j);
#endif
  ContiguousAddressRegion(addr_t lo, addr_t hi) : lo_(lo), hi_(hi) {}
  bool is_valid(addr_t addr) const override {
    return (addr >= lo_) && (addr < hi_);
  }
 private:
  addr_t lo_, hi_;
};

class Platform {
 public:
#ifdef ENABLE_JSON
  static Platform from_json(nlohmann::json j);
#endif
  Platform() : protocol_(Protocol::INVALID) {}
  Platform(Protocol::type protocol) : protocol_(protocol) {}
  
  Protocol::type protocol() const { return protocol_; }
  
  void add_agent(id_t id);
  void add_snoop_filter(id_t id, std::shared_ptr<AddressRegion>&& ar);
  void add_memory(id_t id);

  bool is_valid_agent_id(id_t id) const;
  bool is_valid_snoop_filter_id(id_t id) const;
  id_t get_snoop_filter_id(addr_t addr) const;

  id_t memory_id() const;

 private:
  std::set<id_t> agent_ids_;
  std::map<id_t, std::shared_ptr<AddressRegion> > snoop_filters_;
  Protocol::type protocol_;
  id_t memory_id_;
};

}  // namespace ccm

#endif
