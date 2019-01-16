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

#ifndef __SRC_LOG_HPP__
#define __SRC_LOG_HPP__

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include "sim.hpp"

namespace ccm {

// clang-format off
#define LOGLEVELS(__func)                       \
  __func(debug)                                 \
  __func(info)                                  \
  __func(warning)                               \
  __func(error)                                 \
  __func(fatal)
// clang-format on

// clang-format off
enum class LogLevel {
#define __declare_level(__level) __level,
  LOGLEVELS(__declare_level)
#undef __declare_level
};
// clang-format on

const char* to_string(LogLevel ll);

class LoggerScope;

class Logger {
  friend class LoggerScope;

 public:
  Logger();
  ~Logger();

  LoggerScope* top();
  void set_force_flush(bool force_flush = true) { force_flush_ = force_flush; }

 private:
  void log(const LogLevel ll, const std::string& s);
  void log(const LogLevel ll, const char* s);

  std::ostream& os_{std::cout};
  std::unique_ptr<LoggerScope> top_{nullptr};
  bool force_flush_{false};
};

class LoggerScope {
  friend class Loggable;
  friend class Logger;

  static const char SEP;

  LoggerScope(const std::string& s, Logger* logger);

 public:
  std::string path() const { return path_; }
  Logger* logger() const { return logger_; }

  LoggerScope* child_scope(const std::string& leaf);

 private:
  void log(const LogLevel ll, const std::string& s) { logger_->log(ll, s); }
  void log(const LogLevel ll, const char* s) { logger_->log(ll, s); }

  std::string path_;
  Logger* logger_;
  std::unordered_map<std::string, std::unique_ptr<LoggerScope>> scopes_;
};

class Loggable {
 public:
  LoggerScope* logger_scope() const { return scope_; }
  void set_logger_scope(LoggerScope* scope) { scope_ = scope; }

// clang-format off
#define __declare_handler(__level)                       \
  template <typename... ARGS>                            \
  void log_##__level(ARGS&&... args) {                   \
    log(LogLevel::__level, std::forward<ARGS>(args)...); \
  }
  LOGLEVELS(__declare_handler)
#undef __declare_handler
// clang-format on

  virtual Time time() const = 0;

 private:
  template <typename ARG>
  void log_helper(std::ostream& os, ARG arg) {
    os << arg;
  }

  template <typename ARG, typename... ARGS>
  void log_helper(std::ostream& os, ARG arg, ARGS&&... args) {
    log_helper(os << arg, std::forward<ARGS>(args)...);
  }

  template <typename... ARGS>
  void log(LogLevel ll, ARGS&&... args) {
    if (scope_) {
      std::stringstream ss;
      prefix(ss);
      log_helper(ss, std::forward<ARGS>(args)...);
      scope_->log(ll, ss.str());
    }
  }

  void prefix(std::ostream& os);

  LoggerScope* scope_{nullptr};
};

struct StateUpdateLogger {
  StateUpdateLogger() {}

  std::string str() const { return ss.str(); }

  template <typename T>
  void add(const T& t, const std::string& prefix = "") {
    ss << prefix << "'{" << join(t.begin(), t.end()) << "}";
  }

 private:
  std::stringstream ss;
};

}  // namespace ccm

#endif
