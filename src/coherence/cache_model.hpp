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

#include <vector>

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

struct GenericCacheModelOptions {
  bool is_valid() const { return true; }
  
  uint32_t sets_n{1 << 10};
  uint8_t ways_n{1};
  uint8_t line_bytes_n{64};
  EvictionPolicy eviction_policy{EvictionPolicy::Fixed};
};

struct Addr {
  using type = uint64_t;

  Addr() : d_(0) {}
  Addr(type d) : d_(d) {}

  //
  operator type() { return d_; }

  //
  type line(type len) const {
    return d_ >> kernel::log2ceil(len);
  }
  type offset(type len) const {
    return d_ & ((1 << kernel::log2ceil(len)) - 1);
  }
  type tag(type len, type ways_n) {
    return line(len) >> kernel::log2ceil(ways_n);
  }
 private:
  type d_;
};

template<typename T>
class GenericCacheModel {
 public:
  using momento_type = T;
  
  GenericCacheModel(const GenericCacheModelOptions & opts)
      : opts_(opts) {
    ts_.resize(opts_.sets_n * opts_.ways_n);
    invalidate();
  }
  
  //
  bool is_hit(Addr a, momento_type & m) {
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++) {
      if (ts_[i].is_valid && (ts_[i].tag == a.tag(opts_.line_bytes_n, opts_.ways_n))) {
        m = ts_[i].momento;
        return true;
      }
    }
    return false;
  }

  void update(Addr a, const T & t) {
    // TODO
  }
  
  bool requires_eviction(Addr a) const {
    uint8_t valid = 0;
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++)
      if (ts_[i].is_valid)
        valid++;
    return (valid == opts_.ways_n);
  }

  //
  void install(Addr a, const momento_type & m) {
    for (std::size_t i = line_set_base(a); i < line_set_base(a) + opts_.ways_n; i++) {
      if (!ts_[i].is_valid) {
        ts_[i].is_valid = true;
        ts_[i].tag = a.tag(opts_.line_bytes_n, opts_.ways_n);
        ts_[i].momento = m;
      }
    }
  }
  
  void evict(Addr a, momento_type & m) {
    switch (opts_.eviction_policy) {
      case EvictionPolicy::Fixed: {
        CacheEntry & e = ts_[line_set_base(a)];
        m = e.momento;
        e.is_valid = false;
      } break;
      case EvictionPolicy::Random: {
      
      } break;
      case EvictionPolicy::PsuedoLru: {
      
      } break;
      case EvictionPolicy::TrueLru: {
      
      } break;
      default:
        ;
        // TODO
    }
  }

 private:
  std::size_t line_set_base(Addr a) const {
    return opts_.ways_n * a.line(opts_.line_bytes_n);
  }
  
  void invalidate() {
    for (CacheEntry & ce : ts_)
      ce.is_valid = false;
  }    
  
  struct CacheEntry {
    T momento;
    Addr::type tag;
    bool is_valid;
  };
  
  GenericCacheModelOptions opts_;
  std::vector<CacheEntry> ts_;
};

} // namespace ccm

#endif
