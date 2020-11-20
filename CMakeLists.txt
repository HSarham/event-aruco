cmake_minimum_required(VERSION 2.8)
project(event_aruco)

find_package(libcaer REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

set(CMAKE_POSITION_INDEPENDENT_CODE True)

include_directories(${libcaer_INCLUDE_DIRS} 3rdparty ${Qt5Widgets_INCLUDE_DIRS})
add_subdirectory(3rdparty)

add_library(markercandidate markercandidate.cpp)
add_library(epp eventpacketprocessor.cpp)
target_link_libraries(epp lsd markercandidate)

add_library(imageeventsynchronizer imageeventsynchronizer.cpp)
target_link_libraries(imageeventsynchronizer eventcam)


add_library(liveview liveview.cpp)

add_library(eventcam eventcam.cpp)
target_link_libraries(eventcam libcaer::caer liveview)

add_library(filesystem filesystem.cpp)


add_library(idscam idscam.cpp)
find_library(ueye ueye_api)
target_link_libraries(idscam ${ueye} liveview filesystem)

add_executable(rgbe_recorder rgbe_recorder.cpp)
target_link_libraries(rgbe_recorder ${ueye} ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam)

add_executable(dataset_player dataset_player.cpp)
target_link_libraries(dataset_player ${ueye} ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam)

add_executable(process_events process_events.cpp)
target_link_libraries(process_events ${ueye} ${OpenCV_LIBS} ${Qt5Widgets_LIBRARIES} idscam eventcam epp imageeventsynchronizer)


