cmake_minimum_required(VERSION 2.8)
project(ompl-core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
  message(FATAL_ERROR "Require at least gcc-7.0")
endif()
set(CMAKE_CXX_FLAGS "-fPIC -O2 -DNDEBUG")
#set(CMAKE_EXPORT_COMPILE_COMMANDS True)

# OMPL library.
# This contains OMPL stuff packaged into a static library.
include_directories("ompl_core/")
file(GLOB_RECURSE OMPL_LIB_SRC
  "ompl_core/ompl/*.cpp"
)
add_library(ompl_lib STATIC ${OMPL_LIB_SRC})

# Python dependency.
include_directories(${PYTHON_INCLUDE_DIRS})

# OMPL core library. This contains wrapped functions for
# use in python.
file(GLOB_RECURSE OMPL_CORE_SRC
  "ompl_core/ompl_core/*.cpp"
)
add_library(ompl_core SHARED ${OMPL_CORE_SRC})
target_link_libraries(ompl_core ompl_lib)
set_target_properties(
  ompl_core
  PROPERTIES PREFIX ""
             OUTPUT_NAME "ompl_core"
             LINKER_LANGUAGE C)
