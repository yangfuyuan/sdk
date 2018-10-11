cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_driver)
#add_definitions(-std=c++11) # Use C++11

# Policy CMP0023 allows to mix old and new interfaces of target_link_libraries
cmake_policy(SET CMP0023 OLD)
cmake_policy(SET CMP0022 OLD)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW) # MACOSX_RPATH
endif()

string(TOLOWER ${PROJECT_NAME} LIBRARY_NAME)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

#############################################################################
# Find required libraries

#############################################################################
# HAL macros for driver writers.
set( HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

macro( add_to_hal_include_dirs )
foreach( dir ${ARGN} )
  set_property( GLOBAL APPEND PROPERTY P_INCLUDE_DIRS "${dir}" )
  endforeach()
endmacro()


macro( add_to_hal_libraries )
    foreach( lib ${ARGN} )
      # Process targets correctly
      if (TARGET ${lib})
          # If the library is NOT imported, ie is in this project, we
          # want to depend on it directly rather than through its path
          get_target_property(is_lib_imported ${lib} IMPORTED)
          if (NOT ${is_lib_imported})
            set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
          else()
            # For imported targets, we just want to depend on the library directly
            get_target_property(libpath ${lib} LOCATION)
            if (libpath)
                set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${libpath}" )
              # This shouldn't really happen, but let's cover our bases.
            else()
                set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
            endif()
          endif()
      else()			# Just add the direct path/flag to the list
        set_property( GLOBAL APPEND PROPERTY P_LIBRARIES "${lib}" )
      endif()
    endforeach()

endmacro()


macro( add_to_hal_sources )
    file(RELATIVE_PATH _relPath "${HAL_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_src ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_relPath}/${_src}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_SOURCES "${_src}" )
        endif()
    endforeach()
endmacro()

macro( add_to_hal_headers )
    file(RELATIVE_PATH _relPath "${HAL_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    foreach(_hdr ${ARGN})
        if(_relPath)
            set_property( GLOBAL APPEND PROPERTY P_HEADERS "${_relPath}/${_hdr}" )
        else()
            set_property( GLOBAL APPEND PROPERTY P_HEADERS "${_hdr}" )
        endif()
    endforeach()
endmacro()

macro( hal_set_compile_flags file flags )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_SOURCES "${file}" )
    set_property( GLOBAL APPEND PROPERTY COMPILER_OPTS_FLAGS "${flags}" )
endmacro()


macro(subdirlist result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if( NOT child STREQUAL "CMakeFiles" )
      if(IS_DIRECTORY ${curdir}/${child})
          set(dirlist ${dirlist} ${child})
      endif()
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

IF (WIN32)
add_definitions(-DWIN32_LEAN_AND_MEAN)
endif(WIN32)




FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
)
FILE(GLOB SDK_HDRS
  "include/*.h"
)

add_to_hal_sources( ${SDK_SRC} )
add_to_hal_headers( ${SDK_HDRS} )

add_subdirectory(input_lite)
add_subdirectory(serial)
add_subdirectory(sockets)
add_subdirectory(samples)


#############################################################################
# Setup libraries

get_property( INTERNAL_INC  GLOBAL PROPERTY P_INCLUDE_DIRS )
get_property( INTERNAL_LIBS GLOBAL PROPERTY P_LIBRARIES )
get_property( HAL_SOURCES GLOBAL PROPERTY P_SOURCES )
get_property( HAL_HEADERS GLOBAL PROPERTY P_HEADERS )

# this is a horrible hack in order to set compiler flag properties to individual files
get_property( C_O_S GLOBAL PROPERTY COMPILER_OPTS_SOURCES )
get_property( C_O_F GLOBAL PROPERTY COMPILER_OPTS_FLAGS )

list(LENGTH C_O_S len_c_o_s )
math(EXPR len_c_o_s "${len_c_o_s} - 1" )

foreach(val RANGE ${len_c_o_s} )
  list(GET C_O_S ${val} source )
  list(GET C_O_F ${val} flag )
  set_source_files_properties( ${source} PROPERTIES COMPILE_FLAGS ${flag} )
endforeach()

include_directories( ${CMAKE_BINARY_DIR} )
include_directories( ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )
include_directories( ${LIB_INC_DIR} )
include_directories( ${INTERNAL_INC} )
include_directories( include)
include_directories( src)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/serial )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/sockets )

set(HAL_LIBS
  ${INTERNAL_LIBS}
  ${LINK_LIBS}
  CACHE STRING "HAL required libraries"
  )

list( REMOVE_ITEM HAL_LIBS "debug" )
list( REMOVE_ITEM HAL_LIBS "optimized" )

set(HAL_INCLUDES
    ${LIB_INC_DIR}
    ${USER_INC}
    CACHE STRING "HAL required includes" )


add_library(ydlidar_driver SHARED  ${HAL_SOURCES} ${HAL_HEADERS})
add_library(ydlidar_driver_static STATIC  ${HAL_SOURCES} ${HAL_HEADERS})

SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES OUTPUT_NAME "ydlidar_driver")
SET_TARGET_PROPERTIES(ydlidar_driver PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

target_link_libraries( ydlidar_driver ${HAL_LIBS} )
target_link_libraries( ydlidar_driver_static ${HAL_LIBS} )

IF (WIN32)
target_link_libraries(ydlidar_driver setupapi ws2_32)
target_link_libraries(ydlidar_driver_static setupapi ws2_32)
ELSE(WIN32)
target_link_libraries(ydlidar_driver rt pthread)
target_link_libraries(ydlidar_driver_static rt pthread)

ENDIF(WIN32)
