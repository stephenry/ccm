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

#ifndef __CACHE_MODEL_HPP__
#define __CACHE_MODEL_HPP__

#include "common.hpp"
#include "kernel/kernel.hpp"
#include <tuple>
#include <vector>
#include <algorithm>

namespace ccm {

#define EVICTION_POLICIES(__func)               \
  __func(Fixed)                                 \
  __func(Random)                                \
  __func(PsuedoLru)                             \
  __func(TrueLru)

enum class EvictionPolicy {
#define __declare_eviction_policy(p) p,
  EVICTION_POLICIES(__declare_eviction_policy)
#undef __declare_eviction_policy
};
const char * to_string(EvictionPolicy p);

struct CacheOptions {
  uint32_t sets_n{1 << 10};
  uint8_t ways_n{1};
  uint8_t bytes_per_line{64};
  EvictionPolicy eviction_policy{EvictionPolicy::Fixed};
};

class CacheAddressFormat {
 public:
  CacheAddressFormat() {}
  CacheAddressFormat(std::size_t bytes_per_line, std::size_t sets_n)
      : bytes_per_line_(bytes_per_line), sets_n_(sets_n) {
    
    l2c_bytes_per_line_ = kernel::log2ceil(bytes_per_line_);
    mask_bytes_per_line_ = kernel::mask(l2c_bytes_per_line_);

    l2c_sets_n_ = kernel::log2ceil(sets_n_);
  }
  
  addr_t offset(addr_t a) const {
    return (a & mask_bytes_per_line_);
  }

  addr_t set(addr_t a) const {
    return (a >> l2c_bytes_per_line_);
  }

  addr_t tag(addr_t a) const {
    return set(a) >> l2c_sets_n_;
  }

 private:
  std::size_t bytes_per_line_, l2c_bytes_per_line_, mask_bytes_per_line_;
  std::size_t sets_n_, l2c_sets_n_;
};

template<typename T>
class CacheModel {

 public:

  //
  struct Entry {
    uint64_t addr;
    T t;
  };

  //
  struct Set {
    using iterator = typename std::vector<Entry>::iterator;
    
    std::vector<Entry> lines;
  };
  
  CacheModel(const CacheOptions & opts)
      : opts_(opts) {
    fmt_ = CacheAddressFormat{opts_.bytes_per_line, opts_.sets_n};
    sets_.resize(opts_.sets_n);
  }

  CacheAddressFormat address_format() const {
    return fmt_;
  }

  bool is_hit(addr_t a) const {
    const Set & s = sets_[fmt_.set(a)];
    return s.lines.end() != std::find_if(
        s.lines.begin(), s.lines.end(),
        [=](const Entry & e) { return (e.addr == a); });
  }

  bool requires_eviction(addr_t a) const {
    if (is_hit(a))
      return false;
    
    return (set(a).lines.size() == opts_.ways_n);
  }

  void install(addr_t addr, T & t) {
    Set & s = set(addr);
    s.lines.push_back(Entry{addr, t});
  }

  bool install(addr_t a) {
    return true;
  }

  const T & entry(addr_t a) const {
    static T t;
    return t;
  }

  T & entry(addr_t a) {
    static T t;
    return t;
  }

  template<typename CB>
  std::tuple<bool, Entry> select_eviction(addr_t a, CB && cb) const {
    std::vector<Entry> ves = set_evictables(a, cb);

    switch (opts_.eviction_policy) {
      case EvictionPolicy::Fixed:
        return std::make_tuple(!ves.empty(), ves.back());
        break;
      case EvictionPolicy::Random:
      case EvictionPolicy::PsuedoLru:
      case EvictionPolicy::TrueLru:
        // TODO
        break;
    }
    return std::make_tuple(false, Entry{});
  }

  bool evict(addr_t a) const {
    if (!is_hit(a))
      return false;

    const Set & s = set(a);
    s.lines.erase(std::find_if(s.lines.begin(), s.lines.end(),
                              [=](const Entry & e) { return (e.addr = a); }));
    
    return true;
  }
  
 private:
  const Set & set(addr_t a) const {
    return sets_[fmt_.set(a)];
  }

  template<typename CB>
  std::vector<Entry> set_evictables(addr_t a, CB && cb) const {
    std::vector<Entry> es;
    for (const Entry & e : set(a)) {
      if (cb(e))
        es.push_back(e);
    }
    return es;
  }
  
  CacheAddressFormat fmt_;
  CacheOptions opts_;
  std::vector<Set> sets_;
};

} // namespace ccm

#endif
