cmake_minimum_required(VERSION 3.27)

list( APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" )
include(FetchContent)
include(GitVersion)

get_version_from_git()

message(STATUS "version-full: ${PROJECT_GIT_VERSION_FULL}")

# Set some basic project attributes
project (apv2
	VERSION ${PROJECT_VERSION}
	DESCRIPTION "high compression rate algorithm with low system requirements")
# extra compiler warnings
if (CMAKE_C_COMPILER_ID MATCHES ".*Clang.*")
	# using Clang
    SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wunused-variable -Wswitch-default -Wall -Wextra -Werror")
elseif (CMAKE_C_COMPILER_ID MATCHES "GNU")
	# using GCC
    SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wunused-but-set-variable -Wunused-variable -Wswitch-default -Wall -Wextra -Werror")
elseif (CMAKE_C_COMPILER_ID MATCHES "Intel")
	# using Intel C++
elseif (CMAKE_C_COMPILER_ID MATCHES "MSVC")
	# using Visual Studio C++
endif()
if (CMAKE_CXX_COMPILER_ID MATCHES ".*Clang.*")
	# using Clang
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wunused-variable -Wswitch-default -Wall -Wextra -Werror")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	# using GCC
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wunused-but-set-variable -Wunused-variable -Wswitch-default -Wall -Wextra -Werror")
elseif (CMAKE_CXX_COMPILER_ID MATCHES "Intel")
	# using Intel C++
elseif (CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
	# using Visual Studio C++
endif()

if(DEFINED CMAKE_BUILD_TYPE AND (CMAKE_BUILD_TYPE MATCHES "Debug" OR CMAKE_BUILD_TYPE STREQUAL ""))
    if((CMAKE_C_COMPILER_ID MATCHES "Clang|GNU") AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU"))
        add_compile_options(
	        -fsanitize=undefined -fsanitize=address
        )
        add_link_options(
	        -fsanitize=undefined -fsanitize=address
        )
        message(STATUS "Add sanitizer flags")
    endif()
endif()

if((CMAKE_C_COMPILER_ID MATCHES "GNU") AND (CMAKE_CXX_COMPILER_ID MATCHES "GNU"))
    string(REPLACE "-O3" "-march=native -Ofast" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    string(REPLACE "-O3" "-march=native -Ofast" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "Switching to more agressive compiler optimizations for GCC.")
endif()

if((CMAKE_C_COMPILER_ID MATCHES "Clang") AND (CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
    string(REPLACE "-O3" "-march=native -O3 -ffast-math" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    string(REPLACE "-O3" "-march=native -O3 -ffast-math" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
    message(STATUS "Switching to more agressive compiler optimizations for clang.")
endif()

# Optional IPO. Do not use IPO if it's not supported by compiler.
include(CheckIPOSupported)
check_ipo_supported(RESULT result OUTPUT output)
if(result)
  message(STATUS "IPO enabled")
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
  message(WARNING "IPO is not supported: ${output}")
endif()

find_package(Git REQUIRED)

find_package(boost_headers REQUIRED CONFIG)

# Create a version header
configure_file(version.h.in version.h @ONLY)

FetchContent_Declare(esa_matchfinder_external
                    GIT_REPOSITORY "https://github.com/IlyaGrebnov/esa-matchfinder.git"
                    GIT_TAG main
                    PATCH_COMMAND ${GIT_EXECUTABLE} apply --check "${CMAKE_CURRENT_LIST_DIR}/esa_matchfinder.diff"
					SOURCE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/esa-matchfinder"
                    )
FetchContent_MakeAvailable(esa_matchfinder_external)
set(esa_matchfinder_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/esa-matchfinder" "${CMAKE_SOURCE_DIR}/3rdparty/esa-matchfinder/libsais")
set(esa_matchfinder_SOURCE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty/esa-matchfinder")
include_directories(${esa_matchfinder_INCLUDE_DIRS})

add_library(libesa_matchfinder OBJECT )
target_sources( libesa_matchfinder PRIVATE "${esa_matchfinder_SOURCE_DIRS}/esa_matchfinder.c" "${esa_matchfinder_SOURCE_DIRS}/libsais/libsais.c" )

add_subdirectory(src)

