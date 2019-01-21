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

#ifndef __SRC_CACHE_HPP__
#define __SRC_CACHE_HPP__

#include <algorithm>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "common.hpp"
#include "platform.hpp"
#include "utility.hpp"

namespace ccm {
class CacheLine;
class DirectoryEntry;

// clang-format off
#define EVICTION_POLICIES(__func)               \
  __func(Fixed)                                 \
  __func(Random)                                \
  __func(PsuedoLru)                             \
  __func(TrueLru)
// clang-format on

enum class EvictionPolicy {
// clang-format off
#define __declare_eviction_policy(p) p,
  EVICTION_POLICIES(__declare_eviction_policy)
#undef __declare_eviction_policy
  // clang-format on
};
const char* to_string(EvictionPolicy p);

enum class CacheType { FullyAssociative };

struct CacheOptions {
  CacheType type() const { return type_; }

  void set_type(CacheType type) { type_ = type; }

  uint32_t sets_n{1 << 10};
  uint8_t ways_n{1};
  uint8_t bytes_per_line{64};
  EvictionPolicy eviction_policy{EvictionPolicy::Fixed};

 private:
  CacheType type_;
};

class CacheAddressFormat {
 public:
  CacheAddressFormat() {}
  CacheAddressFormat(std::size_t bytes_per_line, std::size_t sets_n)
      : bytes_per_line_(bytes_per_line), sets_n_(sets_n) {
    l2c_bytes_per_line_ = log2ceil(bytes_per_line_);
    mask_bytes_per_line_ = mask(l2c_bytes_per_line_);

    l2c_sets_n_ = log2ceil(sets_n_);
  }

  addr_t offset(addr_t a) const { return (a & mask_bytes_per_line_); }
  addr_t set(addr_t a) const { return (a >> l2c_bytes_per_line_); }
  addr_t tag(addr_t a) const { return set(a) >> l2c_sets_n_; }

 private:
  std::size_t bytes_per_line_, l2c_bytes_per_line_, mask_bytes_per_line_;
  std::size_t sets_n_, l2c_sets_n_;
};

struct CacheVisitor {
  virtual ~CacheVisitor() {}

  virtual void set_id(id_t id) {}
  virtual void add_line(addr_t addr, const CacheLine& cache_line) {}
  virtual void add_line(addr_t addr, const DirectoryEntry& directory_entry) {}
};

template <typename T>
class GenericCache {
 public:
  GenericCache(const CacheOptions& opts) : opts_(opts) {}
  virtual ~GenericCache() {}

  CacheOptions cache_options() const { return opts_; }

  virtual bool requires_eviction(addr_t addr) const = 0;
  virtual bool is_hit(addr_t addr) const = 0;

  virtual T& lookup(addr_t addr) = 0;

  virtual void visit(CacheVisitor* visitor) = 0;
  virtual void install(addr_t addr, const T& t) = 0;
  virtual bool evict(addr_t addr) = 0;

 private:
  const CacheOptions opts_;
};

template <typename T>
class FullyAssociativeCache : public GenericCache<T> {
 public:
  FullyAssociativeCache(const CacheOptions& opts) : GenericCache<T>(opts) {}

  bool requires_eviction(addr_t addr) const override {
    // TODO
    return false;
  }

  bool is_hit(addr_t addr) const override {
    return (cache_.find(addr) != cache_.end());
  }

  T& lookup(addr_t addr) override {
    if (!is_hit(addr))
      throw std::domain_error("Address is not present in cache.");

    return cache_[addr];
  }

  void visit(CacheVisitor* visitor) override {
    for (auto& [addr, t] : cache_) visitor->add_line(addr, t);
  }

  void install(addr_t addr, const T& t) override { cache_[addr] = t; }
  bool evict(addr_t addr) override {
    auto it = cache_.find(addr);
    const bool found = (it != cache_.end());
    if (found) cache_.erase(it);
    return found;
  }

 private:
  std::unordered_map<addr_t, T> cache_;
};

template <typename T>
std::unique_ptr<GenericCache<T> > cache_factory(const CacheOptions& opts) {
  switch (opts.type()) {
    case CacheType::FullyAssociative:
      return std::make_unique<FullyAssociativeCache<T> >(opts);
    default:;
  }
}

}  // namespace ccm

#endif
