# @author Munzir Zafar
# @date Nov 14, 2018

cmake_minimum_required(VERSION 2.6)
project(krang_sim_ach)

find_package(DART 6.3.0 REQUIRED COMPONENTS utils-urdf gui CONFIG)

# Set the top level directory for use in the code
add_definitions(-DTOP_LEVEL_PATH="${CMAKE_SOURCE_DIR}")

option(krang_sim_ach_SYSTEM_EIGEN "Use system-installed version of Eigen" OFF)

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
#set(CMAKE_C_FLAGS --std=gnu99 -g)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# ================================================================================================
# Include and link to libraries

# Include Eigen
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/eigen)

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either
# in /usr/lib or /usr/local/lib
#link_libraries(amino ntcan protobuf-c ach somatic stdc++ filter pcio pthread ncurses dart fcl tinyxml tinyxml2 kore assimp GL)
link_libraries(stdc++ dart GL config4cpp amino protobuf-c ach somatic)
include_directories(/usr/include/dart)

# ================================================================================================
# Build the codebase library and the executables

# Include the project files
include_directories(include)
include_directories(${DART_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/krang-sim-ach)

# Collect the source, script and fortran files
file(GLOB main_source "src/*.cpp")
file(GLOB scripts_source "exe/*.cpp")
list(SORT scripts_source)

# Build the library and set the executables that use it (have to hardcode for now)
add_library(krangsimach SHARED ${main_source})
set(krangsimachDependents krang_sim_ach)

# Build scripts
message(STATUS "\n-- SCRIPTS: ")
foreach(script_src_file ${scripts_source})
	get_filename_component(script_base ${script_src_file} NAME_WE)
	message(STATUS "Adding script ${script_src_file} with base name ${script_base}" )
	add_executable(${script_base} ${script_src_file})
	target_link_libraries(${script_base} krangsimach)
	target_link_libraries(${script_base} ${DART_LIBRARIES} ${wxWidgets_LIBRARIES})
	add_custom_target(${script_base}.run ${script_base} ${ARGN})
endforeach(script_src_file)

# Install
install(TARGETS krangsimach DESTINATION /usr/local/lib)
FILE(GLOB headers "include/krang-sim-ach/*.h" "include/krang-sim-ach/*.hpp")
install(FILES ${headers} DESTINATION /usr/local/include/krang-sim-ach)
install(DIRECTORY cfg DESTINATION /usr/local/share/krang-sim-ach)
install(PROGRAMS bash/krang-sim-interface DESTINATION /etc/init.d)
install(PROGRAMS build/krang-sim-ach DESTINATION /usr/local/bin)

message(STATUS " ")
