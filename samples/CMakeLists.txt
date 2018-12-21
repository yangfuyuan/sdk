
cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

if(APPLE)
        find_library(corefoundation_lib CoreFoundation)
        find_library(cocoa_lib Cocoa)
        set(INPUT_LITE_PLATFORM_LIBS ${corefoundation_lib} ${cocoa_lib})
else()
        find_package(X11 REQUIRED)
        if(!X11_XTest_FOUND)
                message(FATAL_ERROR "X11 extensions are required, but not found!")
        endif()
        if(!X11_Xfixes_LIB)
                message(FATAL_ERROR "X11 fixes extension is required, but not found!")
        endif()
        find_package(Threads REQUIRED)
        set(INPUT_LITE_PLATFORM_LIBS
                ${X11_LIBRARIES}
                ${X11_Xfixes_LIB}
                ${X11_XTest_LIB}
                ${CMAKE_THREAD_LIBS_INIT}
        )

endif()


#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
ADD_EXECUTABLE(${PROJECT_NAME}
               main.cpp)

# Add the required libraries for linking:
IF (WIN32)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ydlidar_driver input_lite)
ELSE()
TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ydlidar_driver input_lite ${INPUT_LITE_PLATFORM_LIBS})

ENDIF()

