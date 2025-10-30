cmake_minimum_required(VERSION 3.1...3.22)

# Pick your compiler
set(CMAKE_C_COMPILER "/usr/local/gcc-15/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/local/gcc-15/bin/g++")
# set(CMAKE_C_COMPILER "clang-21")
# set(CMAKE_CXX_COMPILER "clang++-21")
set(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(
  auto-vectorization
  VERSION 0.0.1
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

# Don't litter
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Prerequisites: Google Benchmark and dependencies
find_package(benchmark REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)

# Tweak these as needed
add_compile_options(
  -DNDEBUG
  # -DDEBUG
  # -march=native
  -O3
  -march=znver4
  -save-temps
  # -g
)

# Disable auto-vectorization for scalar benchmarks
set_source_files_properties(bench_scalar.cpp PROPERTIES COMPILE_FLAGS -fno-tree-vectorize)

# Combine scalar and auto-vectorized code into single benchmark app
add_executable(meeting
  bench_scalar.cpp
  bench_vec.cpp)

target_link_libraries(meeting PRIVATE benchmark::benchmark)
target_link_libraries(meeting PRIVATE tbb)
