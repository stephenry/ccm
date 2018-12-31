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

#include "snoopfilter.hpp"
#include "msi.hpp"

namespace ccm {

SnoopFilter::SnoopFilter(const SnoopFilterOptions & opts)
    : SnoopFilterCommandInvoker(opts), opts_(opts) {
  set_logger_scope(opts.logger_scope());
}

void SnoopFilter::apply(std::size_t t, const Message * m) {
  pending_messages_.push(make_time_stamped(t, m));
}

bool SnoopFilter::eval(Frontier & f) {
  if (!pending_messages_.empty()) {

    TimeStamped<const Message *> head;
    while (pending_messages_.pop(head)) {
      set_time(head.time());

      const Message * message = head.t();
      const Transaction * transaction = message->transaction();
      
      if (!cache_->is_hit(transaction->addr())) {
        DirectoryEntry directory_entry;
        cc_model_->init(directory_entry);
        cache_->install(transaction->addr(), directory_entry);
      }
      
      DirectoryEntry & directory_entry = cache_->lookup(transaction->addr());
      const CoherenceActions actions =
          cc_model_->get_actions(head.t(), directory_entry);
      execute(f, actions, head.t(), directory_entry);
    }
  }
  return is_active();
}

} // namespace ccm
