cmake_minimum_required(VERSION 3.10)
project(ETSP_WCL_Test1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

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

# Executable for TestOptimizer
add_executable(TestOptimizer
        src/View/TestOptimizer.cpp
        src/Model/Arc.cpp
        src/Model/ChargingOption.cpp
        src/Control/CSVReader.cpp
        src/Model/Node.cpp
        src/Control/Optimizer.cpp
        src/Model/Params.cpp
        src/VNS.cpp
)

# Link libraries for TestOptimizer
target_link_libraries(TestOptimizer PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)

# Set MSVC runtime library for TestOptimizer
if(MSVC)
    set_property(TARGET TestOptimizer PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()

# Executable for TestReader
add_executable(TestReader
        src/View/TestReader.cpp
        src/Model/Arc.cpp
        src/Model/ChargingOption.cpp
        src/Control/CSVReader.cpp
        src/Control/Utils.cpp
        src/Model/Node.cpp
        src/Model/Params.cpp
)

# Link libraries for TestReader
target_link_libraries(TestReader PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)

# Set MSVC runtime library for TestReader
if(MSVC)
    set_property(TARGET TestReader PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()

# Executable for TestReader2
add_executable(TestReader2
        src/View/TestReader2.cpp
        src/Model/Arc.cpp
        src/Control/Route.cpp
        src/Model/ChargingOption.cpp
        src/Control/Optimizer.cpp
        src/Control/CSVReader.cpp
        src/Control/Utils.cpp
        src/Model/Node.cpp
        src/Model/Params.cpp
)

# Link libraries for TestReader2
target_link_libraries(TestReader2 PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)

# Set MSVC runtime library for TestReader
if(MSVC)
    set_property(TARGET TestReader2 PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()

add_executable(TestOptimizer2
        src/View/TestOptimizer2.cpp
        src/Model/Arc.cpp
        src/Control/Route.cpp
        src/Model/ChargingOption.cpp
        src/Control/Optimizer.cpp
        src/Control/Utils.cpp
        src/Control/CSVReader.cpp
        src/Model/Node.cpp
        src/Model/Params.cpp
)

# Link libraries for TestReader2
target_link_libraries(TestOptimizer2 PRIVATE
        ${ILOCPLEX_LIBRARY}
        ${CPLEX_LIBRARY}
        ${CONCERT_LIBRARY}
)

# Set MSVC runtime library for TestReader
if(MSVC)
    set_property(TARGET TestOptimizer2 PROPERTY
            MSVC_RUNTIME_LIBRARY "MultiThreadedDLL"
    )
endif()

add_executable(TestVNS
        src/View/TestVNS.cpp
        src/Control/VNSOptimizer.cpp
        src/Control/CSVReader.cpp
        src/Control/Route.cpp
        src/Control/Optimizer.cpp
        src/Control/Neighborhood.cpp
        src/Control/Utils.cpp
        src/Model/Arc.cpp
        src/Model/ChargingOption.cpp
        src/Model/Node.cpp
        src/Model/Params.cpp
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