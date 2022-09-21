cmake_minimum_required(VERSION 3.0)    
project(test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GLIBCXX_USE_C99=1)

#boost
find_package(Boost REQUIRED COMPONENTS thread filesystem)


# PCL
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# user define
aux_source_directory(src DIR_SRCS)
add_library(SRC ${DIR_SRCS})

find_package( OpenCV 3 REQUIRED )
include_directories(include ${OpenCV_INCLUDE_DIRS} )


# main function
add_executable(main main.cpp)
target_link_libraries (main  SRC -lboost_thread ${Boost_LIBRARIES}  ${PCL_LIBRARIES}  )
