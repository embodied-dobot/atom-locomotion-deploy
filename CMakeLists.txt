cmake_minimum_required(VERSION 3.16)
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" UPPER_BUILD_TYPE)
if(UPPER_BUILD_TYPE STREQUAL "DEBUG")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
endif()

add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DPOLICY_DIR="${CMAKE_CURRENT_SOURCE_DIR}/policy")

find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


set(SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/dobot_atom_sdk)

include_directories(
    /usr/local/include/onnxruntime
    include
    ${SDK_DIR}
    ${SDK_DIR}/idl
    ${SDK_DIR}/common
    ${SDK_DIR}/rpc/include
    ${SDK_DIR}/thirdparty/eigen3
    ${SDK_DIR}/thirdparty/yaml/include
    ${SDK_DIR}/thirdparty/DDS/include
)

add_executable(atom_deploy
    src/atom_deploy.cpp
    ${IDL_SOURCES}
)

target_include_directories(atom_deploy PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SDK_INCLUDE_DIR}
)

target_link_libraries(atom_deploy PRIVATE
    ${SDK_DIR}/lib/libcommon.a
    ${SDK_DIR}/lib/libclient_rpc.a
    ${SDK_DIR}/thirdparty/yaml/lib/libyaml-cpp.a
    ${SDK_DIR}/thirdparty/DDS/lib/libddsc.so
    ${SDK_DIR}/thirdparty/DDS/lib/libcycloneddsidl.so
    ${SDK_DIR}/thirdparty/DDS/lib/libdds_security_ac.so
    ${SDK_DIR}/thirdparty/DDS/lib/libdds_security_auth.so
    ${SDK_DIR}/thirdparty/DDS/lib/libdds_security_crypto.so
    Threads::Threads
    Boost::system
    onnxruntime  
    TBB::tbb
)

