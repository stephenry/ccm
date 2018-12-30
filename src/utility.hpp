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

#include <vector>
#include <string>
#include <cmath>
#include <sstream>

namespace ccm {

class PoolBase;

class Poolable {
  friend class PoolBase;
 public:
  virtual void reset() = 0;
  virtual void release() const;
  virtual ~Poolable() {};
  //  private:
  void set_parent (PoolBase * parent) { parent_ = parent; }
  PoolBase * parent_;
};

class PoolBase {
  friend class Poolable;
 public:
  virtual void release(const Poolable * p) = 0;
};

template<typename T>
class Pool : public PoolBase {
 public:
  Pool(std::size_t n = 1, std::size_t m = 16) : m_(m) {
    construct_n(n);
  }
  T * alloc() {
    if (fl_.size() == 0)
      construct_n(m_);
    T * t = fl_.back();
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
  void release(const Poolable * p) override {
    Poolable * nc_p = const_cast<Poolable *>(p);
    
    nc_p->reset();
    fl_.push_back(static_cast<T *>(nc_p));
  };
  std::size_t m_;
  std::vector<std::unique_ptr<T>> ts_;
  std::vector<T *> fl_;
};

template<typename T>
class TimeStamped {
 public:
  TimeStamped() {}
  TimeStamped(std::size_t time, T & t) : t_(t), time_(time) {}
  
  std::size_t time() const { return time_; }
  T t() const { return t_; }

  void set_time(std::size_t time) { time_ = time; }
 private:
  std::size_t time_;
  T t_;
};

template<typename T>
bool operator<(const TimeStamped<T> & lhs, const TimeStamped<T> & rhs) {
  return (lhs.time() < lhs.time());
}

template<typename T>
auto make_time_stamped(std::size_t time, T & t) -> TimeStamped<T> {
  return TimeStamped<T>(time, t);
}


template<typename T>
class Heap {
 public:
  Heap() : is_heap_(true) {}

  std::vector<T> & ts() { return ts_; }
  const std::vector<T> & ts() const { return ts_; }
  
  void clear() {
    ts_.clear();
    is_heap_ = true;
  }

  bool empty() const {
    return ts_.empty();
  }

  bool peak_head(T & t) {
    bool ret = false;
    if (!empty()) {

      if (!is_heap_)
        heapify();

      t = ts_.front();
    }
    return ret;
  }

  void push(const T & t) {
    ts_.push_back(t);
    is_heap_ = false;
  }

  bool pop(T & t) {
    bool ret = false;
    if (!empty()) {

      if (!is_heap_)
        heapify();
        
      t = ts_.front();
      ts_.erase(ts_.begin());
      ret = true;
    }
    return ret;
  }
  
  void heapify() {
    std::make_heap(ts_.begin(), ts_.end());
    is_heap_ = true;
  }
 private:
  bool is_heap_;
  std::vector<T> ts_;
};

namespace detail {

template<typename ARG>
std::ostream & join_helper(std::ostream & os, ARG && arg) {
  return (os << arg);
}

template<typename ARG, typename ... REST>
std::ostream & join_helper(std::ostream & os, ARG && arg, REST && ... rest) {
  return join_helper(os << arg, std::forward<REST>(rest)...);
}

} // namespace detail

template<typename ...ARGS>
std::string join(ARGS && ... args) {
  std::stringstream ss;
  detail::join_helper(ss, std::forward<ARGS>(args)...);
  return ss.str();
}

template<typename FwdIt>
std::string join(FwdIt begin, FwdIt end, const char * SEP = ", ") {
  std::stringstream ss;
  if (begin != end) {
    ss << *begin; ++begin;

    while (begin != end) {
      ss << *SEP << *begin; ++begin;
    }
  }
  return ss.str();
}

class StructRenderer {
 public:
  StructRenderer() {}

  template<typename VALUE>
  void add(const std::string & k, const VALUE & v) {
  }

  void add(const std::string & k, bool v) {
    add(k, v ? "true" : "false");
  }

  void add(const char * k, const char * v) {
    vs_.push_back(join(k, v));
  }

  std::string str() const {
    std::stringstream ss;
    ss << "'{" << join(vs_.begin(), vs_.end()) << "}";
    return ss.str();
  }
 private:
  std::vector<std::string> vs_;
};

template<typename T>
T log2ceil(T t) {
  return static_cast<T>(std::ceil(std::log2(t)));
}

template<typename T, typename = std::is_integral<T> >
T mask(T t) { return (1 << t) - 1; }

} // namespace ccm

#endif

