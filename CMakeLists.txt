cmake_minimum_required(VERSION 3.2.0)
project(kitti_reader)

if(CMAKE_CURRENT_LIST_DIR STREQUAL CMAKE_SOURCE_DIR)
	set(ENV{CMAKE_PREFIX_PATH} "/home/ys/Source/pcl/build/")
	set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/")
	find_package(PCL 1.8 REQUIRED)
	find_package(OpenCV REQUIRED)
	find_package(Boost REQUIRED)
	find_package(Gperftools REQUIRED)

	add_executable(test_reader ./test/display.cpp)
	target_link_libraries(test_reader ${PROJECT_NAME})
endif()

add_library(${PROJECT_NAME}
	src/KittiReader.cpp
	)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
	$<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
	${Boost_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GPERFTOOLS_INCLUDE_DIR}
	)
target_link_libraries(${PROJECT_NAME} PUBLIC
 	${PCL_LIBRARIES}
	${OpenCV_LIBRARIES}
	${GPERFTOOLS_LIBRARIES}
	)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_auto_type)

install(TARGETS ${PROJECT_NAME} EXPORT MyKittiReaderConfig
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(EXPORT MyKittiReaderConfig DESTINATION share/MyKittiReader/cmake)
export(EXPORT MyKittiReaderConfig FILE MyKittiReaderConfig.cmake)
# export(TARGETS ${PROJECT_NAME} FILE MyKittiReaderConfig.cmake)

