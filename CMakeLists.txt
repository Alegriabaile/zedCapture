cmake_minimum_required(VERSION 3.15)
project(zedCapture)
set(CMAKE_CXX_STANDARD 11)

############################### dependencies ###############################################
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

set(SPECIAL_OS_LIBS "")
if(NOT WIN32)
    set(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings -fpermissive)
endif()

find_package(ZED 2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()




################ zedCapture ############################
add_executable(zedCapture zedCapture/zedCapture.cpp)
add_definitions(-std=c++11 -g -O3 )
target_link_libraries(zedCapture
        ${SPECIAL_OS_LIBS}
        ${ZED_LIBS}
        ${OpenCV_LIBRARIES})


############ createPathOrFile #############################
add_executable(createPathOrFile zedCapture/createPathOrFile.cpp)
target_link_libraries(createPathOrFile "")