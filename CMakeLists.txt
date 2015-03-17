#Project
cmake_minimum_required(VERSION 2.4)
project(spsstereo)

#--------------------------------------------------------------------------------------------------
#
#  Warning Options / C++ Language Options / Code Generation Options
#
add_definitions(
   -Wall
   -W
   -Wextra
   -std=c++1y
   -fPIE
)


#--------------------------------------------------------------------------------------------------
#
# Set default cmake build type to Debug (None Debug Release RelWithDebInfo)
#
if( NOT CMAKE_BUILD_TYPE )
   set( CMAKE_BUILD_TYPE "Debug" )
endif()


if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
   add_definitions(
      -fno-optimize-sibling-calls
        -fno-omit-frame-pointer
   )
endif()


#--------------------------------------------------------------------------------------------------
#
#   Add builds definitions
#
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
   add_definitions( -O1 -DDEBUG -g -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
   add_definitions( -O3 -DDEBUG -march=native -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 )
elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
   add_definitions( -O3 -march=native -msse -msse2 -mssse3 -msse4.1 -msse4.2 -ffast-math -ftree-loop-if-convert -funroll-loops -mfpmath=sse )
endif()

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# SPS-Stereo
file(GLOB SPSSTEREO_SRC_FILES "*.cpp")
add_executable(spsstereo ${SPSSTEREO_SRC_FILES})
target_link_libraries(spsstereo png opencv_core opencv_calib3d opencv_imgproc opencv_highgui ${PCL_LIBRARIES})
