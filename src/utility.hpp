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

#ifndef __SRC_UTILITY_HPP__
#define __SRC_UTILITY_HPP__

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

namespace ccm {

class PoolBase;

class Poolable {
  friend class PoolBase;

 public:
  virtual void reset() = 0;
  virtual void release() const;
  virtual ~Poolable(){};

  void set_parent(PoolBase *parent) { parent_ = parent; }

 private:
  PoolBase *parent_;
};

class PoolBase {
  friend class Poolable;

 public:
  virtual void release(const Poolable *p) = 0;
};

template <typename T>
class Pool : public PoolBase {
 public:
  Pool(std::size_t n = 1, std::size_t m = 16) : m_(m) { construct_n(n); }
  T *alloc() {
    if (fl_.size() == 0) construct_n(m_);
    T *t = fl_.back();
    fl_.pop_back();
    return t;
  }

 private:
  void construct_n(std::size_t n) {
    while (n--) {
      std::unique_ptr<T> t = std::make_unique<T>();
      t->set_parent(this);
      fl_.push_back(t.get());
      ts_.push_back(std::move(t));
    }
  }
  void release(const Poolable *p) override {
    Poolable *nc_p = const_cast<Poolable *>(p);

    nc_p->reset();
    fl_.push_back(static_cast<T *>(nc_p));
  };
  std::size_t m_;
  std::vector<std::unique_ptr<T>> ts_;
  std::vector<T *> fl_;
};

template <typename T>
using MinHeap = std::priority_queue<T, std::vector<T>, std::greater<T>>;

template <typename T>
using MaxHeap = std::priority_queue<T>;

namespace detail {

template <typename ARG>
std::ostream &join_helper(std::ostream &os, ARG &&arg) {
  return (os << arg);
}

template <typename ARG, typename... REST>
std::ostream &join_helper(std::ostream &os, ARG &&arg, REST &&... rest) {
  return join_helper(os << arg, std::forward<REST>(rest)...);
}

}  // namespace detail

template <typename... ARGS>
std::string join(ARGS &&... args) {
  std::stringstream ss;
  detail::join_helper(ss, std::forward<ARGS>(args)...);
  return ss.str();
}

template <typename FwdIt>
std::string join(FwdIt begin, FwdIt end, const char *SEP = ", ") {
  std::stringstream ss;
  if (begin != end) {
    ss << *begin;
    ++begin;

    while (begin != end) {
      ss << SEP << *begin;
      ++begin;
    }
  }
  return ss.str();
}

class StructRenderer {
 public:
  StructRenderer() {}

  void add(const std::string &k, const std::string &v) {
    vs_.push_back(join(k, v));
  }

  std::string str() const {
    std::stringstream ss;
    ss << "'{" << ::ccm::join(vs_.begin(), vs_.end()) << "}";
    return ss.str();
  }

 private:
  std::string join(const std::string &k, const std::string &v) const {
    return k + ":" + v;
  }

  std::vector<std::string> vs_;
};

template <typename T>
T log2ceil(T t) {
  return static_cast<T>(std::ceil(std::log2(t)));
}

template <typename T, typename = std::is_integral<T>>
T mask(T t) {
  return (1 << t) - 1;
}

template <typename T, typename = std::is_integral<T>>
T get_range(T & t, const std::size_t msb, const std::size_t lsb = 0) {
  return (t >> lsb) & mask(msb - lsb);
}

}  // namespace ccm

#endif
