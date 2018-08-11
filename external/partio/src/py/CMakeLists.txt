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



FIND_PACKAGE(SWIG)
IF(SWIG_FOUND) 
FIND_PACKAGE(PythonLibs)
IF(PYTHONLIBS_FOUND)

INCLUDE(${SWIG_USE_FILE})

EXECUTE_PROCESS( COMMAND python -c "import sys;print"%s.%s"%sys.version_info[0:2]" OUTPUT_VARIABLE PYTHON_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)


INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_SWIG_FLAGS "")

# setup output path
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/py)
SET(CMAKE_SWIG_OUTDIR ${PROJECT_BINARY_DIR}/py)

SET_SOURCE_FILES_PROPERTIES(partio.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(partio.i PROPERTIES SWIG_FLAGS "-includeall")
SWIG_ADD_MODULE(partio python partio.i)
SWIG_LINK_LIBRARIES(partio ${PYTHON_LIBRARIES} ${ZLIB_LIBRARY} partio)

SET(PYTHON_DEST "lib64/python${PYTHON_VERSION}/site-packages" )
INSTALL(TARGETS _partio DESTINATION ${PYTHON_DEST})
INSTALL(FILES ${CMAKE_BINARY_DIR}/${outdir}/src/py/partio.py  DESTINATION ${PYTHON_DEST})

ENDIF(PYTHONLIBS_FOUND)
ENDIF(SWIG_FOUND)

