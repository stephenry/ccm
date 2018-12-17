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

#ifndef __OBJECT_HPP__
#define __OBJECT_HPP__

#include "context.hpp"
#include "log.hpp"
#include <string>
#include <optional>

namespace ccm::kernel {

class Module;
class Logger;

class Object {
  static const char SEP;

  friend class Module;
  friend class Process;
  
  Object(const Context & context, const std::string & name = "Unnamed")
      : context_(context), name_(name)
  {}
 public:
  virtual ~Object() {}
  std::string name() const { return name_; }
  virtual void set_name(const std::string & name) { name_ = name; }
  virtual void set_parent(Module * parent) { parent_ = parent; }
  virtual void set_logger(Logger * logger) { logger_ = logger; }
  std::string path();
  std::string prefix();
  Context & context() { return context_; }

 protected:
#define DECLARE_LOGGER(__suffix, __enum)                                \
  template<typename ... ARGS> void log_ ## __suffix(ARGS && ... args) { \
  if (logger_)                                                          \
    logger_->log(__enum, prefix(), std::forward<ARGS>(args)...);        \
  }
  DECLARE_LOGGER(fatal, LogLevel::Fatal)
  DECLARE_LOGGER(error, LogLevel::Error)
  DECLARE_LOGGER(warning, LogLevel::Warning)
  DECLARE_LOGGER(info, LogLevel::Info)
  DECLARE_LOGGER(debug, LogLevel::Debug)
#undef DECLARE_LOGGER

 private:
  Context context_;
  std::string name_;
  std::optional<std::string> path_;
  Module * parent_{nullptr};
  Logger * logger_{nullptr};
};

} // namespace ccm::kernel

#endif
