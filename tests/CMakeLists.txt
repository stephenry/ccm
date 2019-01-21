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

macro (build_common_library)
  add_library(testcommon testcommon.cpp)
  target_include_directories(testcommon PUBLIC ${CMAKE_SOURCE_DIR}/src)
  target_include_directories(testcommon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(testcommon ccm)
endmacro ()

macro (create_test fn)
  add_executable(${fn} ${fn}.cpp)
  target_link_libraries(${fn} ccm gtest_main gtest pthread testcommon)
  target_include_directories(${fn} PUBLIC ${CMAKE_SOURCE_DIR}/src)
  add_test(NAME ${fn} COMMAND ${fn})
endmacro ()

build_common_library()

create_test(msi)
add_subdirectory(mesi)
create_test(mosi)

