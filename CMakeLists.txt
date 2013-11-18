cmake_minimum_required(VERSION 2.8.3)
project(baxter_barcode_pickup)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	image_transport
	cv_bridge
)

catkin_package()

## Declare a cpp executable
add_executable(PlayBagVideo src/PlayBagVideo.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(PlayBagVideo baxter_barcode_pickup_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(PlayBagVideo
  ${catkin_LIBRARIES}
)

