cmake_minimum_required(VERSION 3.16)
project(GenerativeTopologyOptimizer VERSION 1.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(topology_optimizer
    src/main.cpp
    src/TopologyOptimizer.cpp
)
target_link_libraries(topology_optimizer PRIVATE Eigen3::Eigen)

# Automated unit tests
enable_testing()
add_executable(test_optimizer
    tests/test_optimizer.cpp
    src/TopologyOptimizer.cpp
)
target_link_libraries(test_optimizer PRIVATE Eigen3::Eigen)
add_test(NAME TestTopologyOptimizer COMMAND test_optimizer)
