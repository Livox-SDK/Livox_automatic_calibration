CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)

PROJECT(livox_calibration)
set(CMAKE_BUILD_TYPE Release) # this is Important ! ! ! ! !

find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS} include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FILE(GLOB SRCS0 src/mapping/*.cpp)
FILE(GLOB SRCS1 src/calibration/*.cpp)
FILE(GLOB SRCS2 src/ransac/*.cpp)

add_executable (mapping  ${SRCS0} )
target_link_libraries (mapping ${PCL_LIBRARIES} ${LIBS})

add_executable (calibration  ${SRCS1} )
target_link_libraries (calibration ${PCL_LIBRARIES} ${LIBS})

add_executable (fitline  ${SRCS2} )
target_link_libraries (fitline ${PCL_LIBRARIES} ${LIBS})
