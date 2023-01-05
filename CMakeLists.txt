cmake_minimum_required(VERSION 3.8)
project(event_aruco)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(libcaer)
find_package(OpenCV 4.6 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE True)

if (libcaer_FOUND)
    add_compile_options(-DWITH_LIBCAER)
endif()


include_directories(${libcaer_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 3rdparty ${Qt5Widgets_INCLUDE_DIRS})
add_subdirectory(3rdparty)

add_library(markercandidate markercandidate.cpp)
add_library(epp eventpacketprocessor.cpp)
target_link_libraries(epp lsd markercandidate)

add_library(imageeventsynchronizer imageeventsynchronizer.cpp)
target_link_libraries(imageeventsynchronizer eventcam)


add_library(liveview liveview.cpp)

add_library(eventcam eventcam.cpp)
target_link_libraries(eventcam libcaer::caer liveview)

add_library(filesystem_extra filesystem.cpp)


add_library(idscam idscam.cpp)

find_library(ueye ueye_api)
if (ueye_FOUND)
    add_compile_options(-DWITH_UEYE)
    set(ueye_lib ${ueye})
endif()

target_link_libraries(idscam ${ueye_lib} liveview filesystem_extra)

if (ueye_FOUND)
    add_executable(rgbe_recorder rgbe_recorder.cpp)
    target_link_libraries(rgbe_recorder ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam)
endif()

add_executable(dataset_player dataset_player.cpp)
target_link_libraries(dataset_player ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam)

add_executable(process_events process_events.cpp)
target_link_libraries(process_events ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam epp imageeventsynchronizer)


