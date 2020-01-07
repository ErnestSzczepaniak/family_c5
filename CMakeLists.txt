#cmake_version
cmake_minimum_required(VERSION 3.15)

#includes
include(cmake/CMakeConfigBasic.cmake)

include(${CMAKE_CURRENT_SOURCE_DIR}/../designer/CMakeToolchain.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../designer/CMakeFunctions.cmake)

include(cmake/CMakeConfigCompilation.cmake)

project(${name_library} LANGUAGES CXX C ASM)

include(${CMAKE_CURRENT_SOURCE_DIR}/../designer/CMakeConfigGeneric.cmake)


