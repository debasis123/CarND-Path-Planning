project(Path_Planning)

cmake_minimum_required(VERSION 3.14)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(sources
    src/main.cpp
    src/helpers.cpp
    src/simulation_data.cpp
    src/path_planning.cpp
    src/prediction.cpp
    src/behavior_planning.cpp
    src/trajectory_generation.cpp
    src/cost_functions.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif()

add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
