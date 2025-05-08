cmake_minimum_required(VERSION 3.30)
project(ETSP_WCL_Test13)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

add_executable(ETSP_WCL_Test11 main.cpp)

# CPLEX directory
set(CPLEX_DIR "D:/Program Files/IBM/ILOG/CPLEX_Studio2211")

# Link directories for CPLEX and Concert libraries
link_directories(
        "${CPLEX_DIR}/cplex/lib/x64_windows_msvc14/stat_mda"
        "${CPLEX_DIR}/concert/lib/x64_windows_msvc14/stat_mda"
)

# Find CPLEX libraries
find_library(CPLEX_LIBRARY NAMES cplex2211 PATHS "${CPLEX_DIR}/cplex/lib/x64_windows_msvc14/stat_mda")
find_library(CONCERT_LIBRARY NAMES concert PATHS "${CPLEX_DIR}/concert/lib/x64_windows_msvc14/stat_mda")
find_library(ILOCPLEX_LIBRARY NAMES ilocplex PATHS "${CPLEX_DIR}/cplex/lib/x64_windows_msvc14/stat_mda")

# Check if libraries were found
if(NOT CPLEX_LIBRARY)
    message(FATAL_ERROR "Could not find cplex2211.lib")
endif()
if(NOT CONCERT_LIBRARY)
    message(FATAL_ERROR "Could not find concert.lib")
endif()
if(NOT ILOCPLEX_LIBRARY)
    message(FATAL_ERROR "Could not find ilocplex.lib")
endif()

# Include directories
include_directories(
        "${CPLEX_DIR}/cplex/include"
        "${CPLEX_DIR}/concert/include"
        "${CMAKE_SOURCE_DIR}/Test1/src"
)

# MSVC-specific compiler flags
if(MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD /O2 /DNDEBUG /D_ITERATOR_DEBUG_LEVEL=0")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd /Zi /Ob0 /Od /RTC1 /D_ITERATOR_DEBUG_LEVEL=2")
endif()

# Copy data files to build directory
file(COPY "${CMAKE_SOURCE_DIR}/data/Input/" DESTINATION "${CMAKE_BINARY_DIR}/data")

add_executable(TestInput
    TestInput.cpp
    src/Utils.cpp
    src/Route.cpp
    src/Graph.cpp
    src/Parameters.cpp
)


add_executable(TestVNS
        TestVNS.cpp
        src/VNS.cpp
        src/Route.cpp
        src/Optimizer.cpp
        src/Utils.cpp
        src/Parameters.cpp
        src/Graph.cpp
)

# Link libraries for TestVNS
target_link_libraries(TestVNS PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)


# Định nghĩa _ITERATOR_DEBUG_LEVEL=0 để tắt kiểm tra iterator debug
target_compile_definitions(TestVNS PRIVATE _ITERATOR_DEBUG_LEVEL=0)

# Set MSVC runtime library for TestReader
if(MSVC)
    set_property(TARGET TestVNS PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()

add_executable(TestMILP
        TestMILP.cpp
        src/Route.cpp
        src/Optimizer.cpp
        src/Utils.cpp
        src/Parameters.cpp
        src/Graph.cpp
)

# Link libraries for TestVNS
target_link_libraries(TestVNS PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)


# Định nghĩa _ITERATOR_DEBUG_LEVEL=0 để tắt kiểm tra iterator debug
target_compile_definitions(TestVNS PRIVATE _ITERATOR_DEBUG_LEVEL=0)

# Set MSVC runtime library for TestReader
if(MSVC)
    set_property(TARGET TestVNS PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()