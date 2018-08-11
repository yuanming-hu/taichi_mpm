# PARTIO SOFTWARE
# Copyright 2010 Disney Enterprises, Inc. All rights reserved
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in
# the documentation and/or other materials provided with the
# distribution.
# 
# * The names "Disney", "Walt Disney Pictures", "Walt Disney Animation
# Studios" or the names of its contributors may NOT be used to
# endorse or promote products derived from this software without
# specific prior written permission from Walt Disney Pictures.
# 
# Disclaimer: THIS SOFTWARE IS PROVIDED BY WALT DISNEY PICTURES AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE ARE DISCLAIMED.
# IN NO EVENT SHALL WALT DISNEY PICTURES, THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND BASED ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.

include_directories(
   ${CMAKE_SOURCE_DIR}/src/lib
)

IF(GLUT_FOUND AND OPENGL_FOUND)
    ADD_EXECUTABLE(partview partview.cpp)
    target_link_libraries(partview ${PARTIO_LIBRARIES} ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
    install(TARGETS  partview DESTINATION bin)
ENDIF(GLUT_FOUND AND OPENGL_FOUND)

ADD_EXECUTABLE(partinfo partinfo.cpp)
target_link_libraries(partinfo ${PARTIO_LIBRARIES})

ADD_EXECUTABLE(partconv partconv.cpp)
target_link_libraries(partconv ${PARTIO_LIBRARIES})

ADD_EXECUTABLE(partattr partattr.cpp)
target_link_libraries(partattr ${PARTIO_LIBRARIES})

install(TARGETS partattr partconv partinfo DESTINATION bin)

