cmake_minimum_required(VERSION 3.16...3.27)
project(example_minimal LANGUAGES CXX)

add_executable(example_minimal main.cpp)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# Rerun requires at least C++17, but it should be compatible with newer versions.
set_property(TARGET example_minimal PROPERTY CXX_STANDARD 17)

# Link against rerun_sdk.
target_link_libraries(example_minimal PRIVATE rerun_sdk)
