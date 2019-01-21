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

#ifndef __SRC_ACTOR_HPP__
#define __SRC_ACTOR_HPP__

#include "cache.hpp"
#include "log.hpp"
#include "platform.hpp"
#include "sim.hpp"
#include "transaction.hpp"

namespace ccm {

class Message;
class Epoch;
class Context;
class Cursor;
class MessageBuilder;

struct ActorOptions {
  ActorOptions(id_t id, Platform platform) : id_(id), platform_(platform) {}
  id_t id() const { return id_; }
  LoggerScope *logger_scope() const { return logger_scope_; }
  Platform platform() const { return platform_; }

  void set_logger_scope(LoggerScope *logger_scope) {
    logger_scope_ = logger_scope;
  }

 private:
  id_t id_;
  CacheOptions cache_options_;
  LoggerScope *logger_scope_;
  Platform platform_;
};

struct CoherentActor : Loggable {
  CoherentActor(const ActorOptions &opts) : opts_(opts), time_(0) {}
  virtual ~CoherentActor() {}

  Time time() const override { return time_; }
  id_t id() const { return opts_.id(); }

  virtual void apply(TimeStamped<Message *> ts) = 0;
  virtual void eval(Context &ctxt) = 0;
  virtual bool is_active() const = 0;
  virtual void visit_cache(CacheVisitor *cache_visitor) const = 0;

  void set_time(Time time) { time_ = time; }
  void emit_message(Context &context, Cursor &cursor, MessageBuilder &b);

 protected:
  const ActorOptions opts_;

 private:
  Time time_;
};

}  // namespace ccm

#endif
