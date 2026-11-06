cmake_minimum_required(VERSION 3.16)
project(check_cereal LANGUAGES CXX)

find_package(cereal 1.3.2 CONFIG REQUIRED)
get_target_property(cereal_include_dirs cereal::cereal INTERFACE_INCLUDE_DIRECTORIES)
foreach(include_dir IN LISTS cereal_include_dirs)
  if(NOT IS_DIRECTORY "${include_dir}")
    message(FATAL_ERROR "cereal exports a missing include directory: ${include_dir}")
  endif()
endforeach()

add_executable(check-cereal main.cpp)
target_link_libraries(check-cereal PRIVATE cereal::cereal)
target_compile_features(check-cereal PRIVATE cxx_std_11)
