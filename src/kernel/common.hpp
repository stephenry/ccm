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

#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#include <vector>
#include <memory>
#include <iostream>
#include <string>

#define CCM_MACRO_BEGIN do {
#define CCM_MACRO_END   } while (false)

#define CCM_ASSERT(__cond)                                      \
  CCM_MACRO_BEGIN                                               \
  if (!(__cond)) {                                              \
    std::cout << __FILE__ << ":" << __LINE__                    \
              << " assertion failed: " << #__cond << "\n";      \
    std::exit(1);                                               \
  }                                                             \
  CCM_MACRO_END

namespace ccm {

  class PoolBase;
  class Poolable;
  class Scheduler;
  
  class Poolable {
    friend class PoolBase;
  public:
    virtual void reset() = 0;
    virtual void release();
    virtual ~Poolable() {};
    //  private:
    void set_parent (PoolBase * parent) { parent_ = parent; }
    PoolBase * parent_;
  };

  class PoolBase {
    friend class Poolable;
  public:
    virtual void release(Poolable * p) = 0;
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
    void release(Poolable * p) override {
      p->reset();
      fl_.push_back(static_cast<T *>(p));
    };
    std::size_t m_;
    std::vector<std::unique_ptr<T>> ts_;
    std::vector<T *> fl_;
  };

  template<typename T>
  struct StaticResource {
    StaticResource()
      : is_initialized_(false) {
      std::cerr << "Constructing\n";
    }
    ~StaticResource() {
      if (is_initialized_)
        delete t_;
    }
    T& operator*() {
      if (!is_initialized_)
        do_init();
      return *t_;
    }
    T* operator->() {
      if (!is_initialized_)
        do_init();
      return t_;
    }
  private:
    void do_init() {
      t_ = new T{};
      is_initialized_ = true;
    }
    T * t_;
    bool is_initialized_{false};
  };

  class ReferenceCounted {
  public:
    ReferenceCounted()
      : cnt_(1) {}
    void inc() { cnt_++; }
    void dec() {
      if (cnt_-- == 0)
        delete this;
    }
  private:
    std::size_t cnt_;
  };

} // namespace ccm

#endif
