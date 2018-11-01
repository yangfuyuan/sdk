cmake_minimum_required(VERSION 2.8)
include_directories(include)
include_directories(src)

IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.c"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
)

	
ENDIF()

add_subdirectory(samples)

add_library(ydlidar_driver SHARED ${SDK_SRC})
IF (WIN32)
ELSE()
target_link_libraries(ydlidar_driver rt pthread)
ENDIF()                    
