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
#include <functional>
#include "options.hpp"
#ifdef ENABLE_JSON
#  include <nlohmann/json.hpp>
#endif
#include "common.hpp"
#include "utility.hpp"
#include "types.hpp"
#include "random.hpp"

namespace ccm {

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

enum class CacheType {
  DirectMapped,
  SetAssociative,
  FullyAssociative,
  InfiniteCapacity,
  Invalid
};

struct CacheOptions {
#define CACHE_OPTION_FIELDS(__func)                     \
  __func(type, CacheType, CacheType::InfiniteCapacity)  \
  __func(line_bytes, uint8_t, 64)                       \
  __func(ways_n, uint8_t, 4)                            \
  __func(size_bytes, std::size_t, (1<<15))
  
#ifdef ENABLE_JSON
  static CacheOptions from_json(nlohmann::json j);
#endif
  CacheOptions();

#define __declare_etters(__name,  __type, __default)            \
  __type __name() const { return __name ## _; }                 \
  void set_ ## __name(__type __name) { __name ## _ = __name; }
  CACHE_OPTION_FIELDS(__declare_etters)
#undef __declare_etters

 private:
  void reset();

#define __declare_fields(__name, __type, __default)     \
  __type __name ## _{__default};
  CACHE_OPTION_FIELDS(__declare_fields)
#undef __declare_fields
};

class CacheAddressFieldHelper {
#define CACHE_ADDRESS_FIELD_HELPER_FIELDS(__func)       \
  __func(bits_for_line, std::size_t)                    \
  __func(bits_for_set, std::size_t)                     \
  __func(lines_n, std::size_t)                          \
  __func(sets_n, std::size_t)
  
 public:
  CacheAddressFieldHelper(const CacheOptions & opts);

#define __declare_getters(__field, __type)      \
  __type __field() const { return __field ## _; }
  CACHE_ADDRESS_FIELD_HELPER_FIELDS(__declare_getters)
#undef __declare_getters
  
  std::size_t offset(addr_t a) const;
  std::size_t set(addr_t a) const;
  std::size_t tag(addr_t a) const;
  std::size_t base(addr_t a) const;

 private:
  void precompute();
#define __declare_fields(__field, __type)       \
  __type __field ## _;
  CACHE_ADDRESS_FIELD_HELPER_FIELDS(__declare_fields)
#undef __declare_fields
  const CacheOptions & opts_;
};

class CacheableEntity {
#define CACHEABLE_ENTITY_FIELDS(__func)         \
  __func(state, state_t, 0)                     \
  __func(is_valid, bool, false)                 \
  __func(base, addr_t, 0)
 public:

#define __declare_etters(__name, __type, __default)             \
  __type __name() const { return __name ## _; }                 \
  void set_ ## __name(__type __name) { __name ## _ = __name; }
  CACHEABLE_ENTITY_FIELDS(__declare_etters)
#undef __declare_etters

  private:
#define __declare_fields(__name, __type, __default)     \
  __type __name ## _{__default};
  CACHEABLE_ENTITY_FIELDS(__declare_fields)
#undef __declare_fields
};

class CacheLine;
class DirectoryLine;

struct CacheVisitor {
  virtual ~CacheVisitor() {}

  virtual void set_id(id_t id) {}
  virtual void add_line(addr_t addr, const CacheLine& cache_line) {}
  virtual void add_line(addr_t addr, const DirectoryLine& directory_entry) {}
};

template <typename T>
class GenericCache {
 public:
  GenericCache() {}
  virtual ~GenericCache() {}

  virtual bool is_hit(addr_t addr) const = 0;
  virtual bool requires_eviction(addr_t addr) const = 0;
  virtual const T & nominate_evictee(addr_t addr) const = 0;

  virtual const T& lookup(addr_t addr) const = 0;
  virtual T& lookup(addr_t addr) = 0;

  virtual void visit(CacheVisitor* visitor) = 0;
  virtual bool install(addr_t addr, const T& t) = 0;
  virtual bool evict(addr_t addr) = 0;
  virtual void reset() = 0;
};

template<typename T>
class SetAssociativeCache : public GenericCache<T> {
 public:
  SetAssociativeCache(const CacheOptions & opts)
      : GenericCache<T>(), opts_(opts), fields_(opts) {
    lines_.resize(fields_.lines_n());
  }
  bool is_hit(addr_t addr) const override {
    bool found{false};
    enumerate_ways(addr, [&](const T & t) -> bool {
        if (!found && t.is_valid() && (t.base() == fields_.base(addr)))
          found = true;
        return !found;
      });
    return found;
  }
  bool requires_eviction(addr_t addr) const override {
    bool found{false};
    std::size_t ways_used_n{0};
    enumerate_ways(addr, [&](const T & t) -> bool {
        if (found || !t.is_valid())
          return false;

        ++ways_used_n;
        found = (t.base() == fields_.base(addr));
        return !found;
      });
    return !found && (ways_used_n == opts_.ways_n());
  }
  const T & nominate_evictee(addr_t addr) const override {
    Random::UniformRandomInterval rnd{opts_.ways_n() - 1, 0};

    const T * evictee{std::addressof(t_invalid_)};
    int evict_way{rnd()};
    enumerate_ways(addr, [&](const T & t) -> bool {
        const bool sel_evictee = !t.is_valid() || (evict_way-- == 0);
        if (sel_evictee)
          evictee = std::addressof(t);
        return !sel_evictee;
      });
    return *evictee;
  }
  virtual bool evict(addr_t addr) override {
    bool did_evict{false};
    enumerate_ways(addr, [&](T & t) -> bool {
        if (!t.is_valid())
          return true;

        did_evict = (t.base() == fields_.base(addr));
        if (did_evict)
          t.set_is_valid(false);
        return !did_evict;
      });
    return did_evict;
  }
  const T & lookup(addr_t addr) const override {
    const T * line{std::addressof(t_invalid_)};
    enumerate_ways(addr, [&](const T & t) -> bool {
        if (!t.is_valid())
          return true;

        if (t.base() == fields_.base(addr))
          line = std::addressof(t);

        return (line == std::addressof(t_invalid_));
      });
    return *line;
  }
  T & lookup(addr_t addr) override {
    T * line{std::addressof(t_invalid_)};
    enumerate_ways(addr, [&](T & t) -> bool {
        if (!t.is_valid())
          return true;

        if (t.base() == fields_.base(addr))
          line = std::addressof(t);

        return (line == std::addressof(t_invalid_));
      });
    return *line;
  }
  virtual void visit(CacheVisitor* visitor) override {
    for (T & t : lines_)
      visitor->add_line(t.base(), t);
  }
  virtual bool install(addr_t addr, const T& t) override {
    const bool did_hit = is_hit(addr);
    bool did_install{false};
    T t_install{t};
    t_install.set_is_valid(true);
    t_install.set_base(fields_.base(addr));
    enumerate_ways(addr, [&](T & slot) -> bool {
        bool do_continue{true};
        if (!did_install && !slot.is_valid()) {
          slot = t_install;
          did_install = true;
        } else if (did_install && slot.is_valid() && (slot.base() == fields_.base(addr))) {
          slot = t_install;
          did_install = true;
        }
        return do_continue;
      });
    return did_install;
  }
  void reset() override {
    for (T & t : lines_)
      t.set_is_valid(false);
  }
 private:
  template<typename FN>
  void enumerate_ways(addr_t addr, FN && f) const {
    const std::size_t way_base = opts_.ways_n() * fields_.set(addr);
    for (std::size_t way_off = 0; way_off < opts_.ways_n(); ++way_off) {
      const T & line = lines_[way_base + way_off];
      if (!f(std::cref(line)))
        return;
    }
  }
  template<typename FN>
  void enumerate_ways(addr_t addr, FN && f) {
    const std::size_t way_base = opts_.ways_n() * fields_.set(addr);
    for (std::size_t way_off = 0; way_off < opts_.ways_n(); ++way_off) {
      T & line = lines_[way_base + way_off];
      if (!f(std::ref(line)))
        return;
    }
  }
  T t_invalid_;
  std::vector<T> lines_;
  const CacheAddressFieldHelper fields_;
  const CacheOptions opts_;
};

template <typename T>
class FullyAssociativeCache : public GenericCache<T> {
 public:
  FullyAssociativeCache(const CacheOptions& opts)
      : GenericCache<T>(), opts_(opts), fields_(opts) {}

  bool requires_eviction(addr_t addr) const override {
    // TODO
    return false;
  }
  const T & nominate_evictee(addr_t addr) const override { return t_invalid_; }
  bool is_hit(addr_t addr) const override {
    return (cache_.find(addr) != cache_.end());
  }
  const T& lookup(addr_t addr) const override {
    return cache_.at(addr);
  }
  T& lookup(addr_t addr) override {
    if (!is_hit(addr))
      throw std::domain_error("Address is not present in cache.");

    return cache_[addr];
  }
  void visit(CacheVisitor* visitor) override {
    for (auto& [addr, t] : cache_) visitor->add_line(addr, t);
  }
  bool install(addr_t addr, const T& t) override {
    cache_[addr] = t;
    return true;
  }
  bool evict(addr_t addr) override {
    auto it = cache_.find(addr);
    const bool found = (it != cache_.end());
    if (found) cache_.erase(it);
    return found;
  }
  void reset() override { cache_.clear(); }
 private:
  T t_invalid_;
  std::unordered_map<addr_t, T> cache_;
  const CacheAddressFieldHelper fields_;
  const CacheOptions opts_;
};

template <typename T>
class InfiniteCapacityCache : public GenericCache<T> {
 public:
  InfiniteCapacityCache(const CacheOptions& opts)
      : GenericCache<T>(), opts_(opts), fields_(opts) {}

  bool requires_eviction(addr_t addr) const override { return false; }
  const T & nominate_evictee(addr_t addr) const override { return t_invalid_; }
  bool is_hit(addr_t addr) const override {
    return (cache_.find(addr) != cache_.end());
  }
  const T& lookup(addr_t addr) const override {
    return cache_.at(addr);
  }
  T& lookup(addr_t addr) override {
    if (!is_hit(addr))
      throw std::domain_error("Address is not present in cache.");

    return cache_[addr];
  }
  void visit(CacheVisitor* visitor) override {
    for (auto& [addr, t] : cache_) visitor->add_line(addr, t);
  }
  bool install(addr_t addr, const T& t) override {
    cache_[addr] = t;
    cache_[addr].set_is_valid(true);
    return true;
  }
  bool evict(addr_t addr) override {
    auto it = cache_.find(addr);
    const bool found = (it != cache_.end());
    if (found) cache_.erase(it);
    return found;
  }
  void reset() override { cache_.clear(); }
 private:
  T t_invalid_;
  std::unordered_map<addr_t, T> cache_;
  const CacheAddressFieldHelper fields_;
  const CacheOptions opts_;
};

template <typename T>
std::unique_ptr<GenericCache<T> > cache_factory(const CacheOptions& opts) {
  switch (opts.type()) {
    case CacheType::DirectMapped: {
      // A direct mapped cache is simply a set-associative cache with
      // a way count of 1.
      //
      CacheOptions dm_opts{opts};
      dm_opts.set_ways_n(1);
      return std::make_unique<SetAssociativeCache<T> >(dm_opts);
    } break;
    case CacheType::SetAssociative:
      return std::make_unique<SetAssociativeCache<T> >(opts);
      break;
    case CacheType::FullyAssociative:
      return std::make_unique<FullyAssociativeCache<T> >(opts);
      break;
    case CacheType::InfiniteCapacity:
      return std::make_unique<InfiniteCapacityCache<T> >(opts);
    default:
      return nullptr;
      break;
  }
}

}  // namespace ccm

#endif
