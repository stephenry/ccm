##========================================================================== //
## Copyright (c) 2018, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##========================================================================== //

set(SRCS
  platform.cpp
  actor.cpp
  utility.cpp
  interconnect.cpp
  message.cpp
  log.cpp
  agent.cpp
  snoopfilter.cpp
  sim.cpp
  transaction.cpp
  random.cpp
  memory.cpp
  protocol.cpp
  cache.cpp
  msi.cpp
  )

if (ENABLE_MESI)
  list(APPEND SRCS mesi.cpp)
endif ()
if (ENABLE_MOSI)
  list(APPEND SRCS mosi.cpp)
endif ()
if (ENABLE_JSON)
  list(APPEND SRCS builder.cpp)
endif ()

configure_file(options.hpp.in options.hpp)
add_library(ccm STATIC ${SRCS})
target_include_directories(ccm PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
if (ENABLE_JSON)
  target_link_libraries(ccm PUBLIC nlohmann_json::nlohmann_json)
endif ()
