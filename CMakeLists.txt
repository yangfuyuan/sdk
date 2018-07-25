cmake_minimum_required(VERSION 2.8)
#add_definitions(-std=c++11) # Use C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(include)
include_directories(src)
include_directories(matrix)


IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
  "matrix/matrix/*.hpp"

)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
  "matrix/matrix/*.hpp"

)

	
ENDIF()

add_subdirectory(samples)
add_library(ydlidar_driver SHARED ${SDK_SRC})
add_library(ydlidar_driver_static STATIC ${SDK_SRC})

SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES OUTPUT_NAME "ydlidar_driver")

SET_TARGET_PROPERTIES(ydlidar_driver PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(ydlidar_driver_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
IF (WIN32)
ELSE()
target_link_libraries(ydlidar_driver rt pthread)
target_link_libraries(ydlidar_driver_static rt pthread)
ENDIF()                    
