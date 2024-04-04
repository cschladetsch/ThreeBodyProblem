cmake_minimum_required(VERSION 3.12)
project(ThreeBodySimulation VERSION 1.0 DESCRIPTION "Three Body Problem Simulation using Verlet Integration with Boost and SDL2")

# Use C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find SDL2
set(SDL2_DIR "/c/SDL2/SDL2-devel-2.30.2-VC/SDL2-2.30.2/")
include_directories("/c/SDL2/SDL2-devel-2.30.2-VC/SDL2-2.30.2/include")
link_directories("/c/SDL2/SDL2-devel-2.30.2-VC/SDL2-2.30.2/lib/x64/SDL2")

# Find Boost, specify the components you need
#find_package(Boost 1.84.0 REQUIRED COMPONENTS filesystem system)
#include_directories(${Boost_INCLUDE_DIR})

# Define the executable
add_executable(${PROJECT_NAME} main.cpp)

# Link with SDL2 and Boost libraries
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} SDL2main ${Boost_LIBRARIES})

