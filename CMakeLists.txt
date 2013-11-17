cmake_minimum_required(VERSION 2.8.3)
project(baxter_barcode_pickup)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	image_transport
	cv_bridge
)

catkin_package()

add_executable(video_to_bag src/video_to_bag.cpp)

target_link_libraries(video_to_bag ${catkin_LIBRARIES})

