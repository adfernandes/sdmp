cmake_minimum_required(VERSION 3.13.4)

# ----------------------------------------------------------------------------
# Project Setup

project(sdmp VERSION 0.0
    DESCRIPTION "The Simple Droid Motion Planner"
    LANGUAGES CXX)

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build Shared Libs")

set(PROJECT_NAMESPACE "droid::")

# ----------------------------------------------------------------------------
# Set the project version, a major version of "0" indicates git-prerelease

find_package(Git REQUIRED)

execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    OUTPUT_VARIABLE GIT_COMMIT_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE)

execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --dirty --broken
    OUTPUT_VARIABLE GIT_DESCRIPTION
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(${GIT_DESCRIPTION} MATCHES "^[1-9]+[0-9]*[.][0-9]+$")
    set(PROJECT_VERSION ${GIT_DESCRIPTION})
endif()

string(REGEX REPLACE "[.]" ";" PROJECT_VERSION_LIST ${PROJECT_VERSION})
list(GET PROJECT_VERSION_LIST 0 PROJECT_VERSION_MAJOR)
list(GET PROJECT_VERSION_LIST 1 PROJECT_VERSION_MINOR)

# TODO: Use 'configure_file' to set the version numbers in the
#       library header file so they can be queried at runtime.

# ----------------------------------------------------------------------------
# Find all dependencies

find_package(ompl 1 CONFIG REQUIRED)

find_package(Boost 1.67 REQUIRED)
find_package(Catch2 2.12 REQUIRED)
find_package(Doxygen 1.8 REQUIRED)
find_package(Protobuf 3.6 REQUIRED)

# ----------------------------------------------------------------------------
# Set the sources for each target

set(LIBRARY_HEADERS include/sdmp.hpp)
set(LIBRARY_SOURCES library/sdmp.cpp)
set(LIBRARY_PROTOCS library/sdmp.proto)

set(TEST_SOURCES test/test.cpp)

set(APPLICATION_SOURCES application/main.cpp)

# ----------------------------------------------------------------------------
# The main library target

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${LIBRARY_PROTOCS})

add_library(${PROJECT_NAME} ${LIBRARY_HEADERS} ${LIBRARY_SOURCES} ${PROTO_SRCS} ${PROTO_HDRS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER ${LIBRARY_HEADERS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIR})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    PRIVATE
        ${OMPL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/library)

target_link_libraries(${PROJECT_NAME} PRIVATE ${OMPL_LIBRARIES} ${Boost_LIBRARIES} protobuf::libprotobuf)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-export
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${LIBRARY_PROTOCS} ${PROTO_HDRS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

install(EXPORT ${PROJECT_NAME}-export
  FILE ${PROJECT_NAME}.cmake
  NAMESPACE ${PROJECT_NAMESPACE}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake)

# ----------------------------------------------------------------------------
# The documentation target

set(DOXYGEN_EXCLUDE_PATTERNS "*.pb.*")
set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

doxygen_add_docs(${PROJECT_NAME}-doxygen ${PROJECT_SOURCE_DIR} ALL)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/doc/${PROJECT_NAME})

# ----------------------------------------------------------------------------
# The main "test" target

add_executable(${PROJECT_NAME}-test ${TEST_SOURCES})

target_compile_features(${PROJECT_NAME}-test PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}-test
    PRIVATE ${PROJECT_SOURCE_DIR}/test)

target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} protobuf::libprotobuf Catch2::Catch2)

include(CTest)
include(Catch)

catch_discover_tests(${PROJECT_NAME}-test)

# ----------------------------------------------------------------------------
# The main command line application target

add_executable(${PROJECT_NAME}-application ${APPLICATION_SOURCES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

set_target_properties(${PROJECT_NAME}-application PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}-application PRIVATE ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}-application
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

# ----------------------------------------------------------------------------
