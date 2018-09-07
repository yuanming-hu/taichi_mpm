set(TAICHI_PROJECT_NAME "mpm")

if ("${TARGET_ARCHITECTURE}" MATCHES "sandy-bridge")
    message("No -DHASWELL for sandy-bridge")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHASWELL")
endif()

file(GLOB PROJECT_SOURCES
        "src/*.cpp" "external/SPGrid/*/*.cpp", "src/async/*.cpp")

add_library(taichi_${TAICHI_PROJECT_NAME} SHARED ${PROJECT_SOURCES})
include_directories(external/partio/include)
include_directories(external/)
include_directories(external/libccd/src)

set(TAICHI_PROJECT_LIBRARIES ${TAICHI_PROJECT_LIBRARIES} ${TAICHI_PROJECT_NAME} PARENT_SCOPE)
target_link_libraries(taichi_${TAICHI_PROJECT_NAME} ${CORE_LIBRARY_NAME})
target_link_libraries(taichi_${TAICHI_PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/external/partio/lib/libpartio.a)

find_package(ZLIB)
if (ZLIB_FOUND)
    target_link_libraries(taichi_${TAICHI_PROJECT_NAME} z)
endif(ZLIB_FOUND)

target_link_libraries(taichi_${TAICHI_PROJECT_NAME} ccd)

add_subdirectory(external/libccd)

