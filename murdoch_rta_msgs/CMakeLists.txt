cmake_minimum_required(VERSION 2.8.3)

project(murdoch_rta_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation nav_msgs std_msgs)

add_message_files(
	DIRECTORY msg
	FILES BaseMessage.msg
              Task.msg
              Resource.msg
              Auction.msg
              Bid.msg
	      Boolean.msg
)

add_service_files(
	DIRECTORY srv
	FILES setTask.srv
	      getResources.srv
	      getTaskStatus.srv
)

generate_messages(DEPENDENCIES geometry_msgs murdoch_rta_msgs nav_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}

)
