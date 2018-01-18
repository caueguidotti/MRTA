cmake_minimum_required(VERSION 2.8.3)

project(murdoch_rta)

find_package(catkin REQUIRED COMPONENTS pluginlib roscpp geometry_msgs murdoch_rta_msgs)

## Find Boost (headers only)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS pluginlib roscpp geometry_msgs murdoch_rta_msgs
    )

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(H_FILES
    # Utilities
    include/utils/time.h
    include/utils/task_monitoring/BaseTaskMonitor.h
    include/utils/task_generator/BaseTaskGenerator.h
    include/utils/robot/BaseRobot.h

    # Plugins
    include/murdoch_rta/Base.h
    include/murdoch_rta/UtilityCalculator.h

    # Murdoch
    include/murdoch_rta/Robot.h

    # Exceptions
    include/murdoch_rta/MurdochExceptions.h

    # Murdoch/Communicator
    include/murdoch_rta/communicator/Listener.h
    include/murdoch_rta/communicator/Task.h

    # Murdoch/Communicator/Messages
    include/murdoch_rta/communicator/messages/Message.h
    include/murdoch_rta/communicator/messages/MessageType.h
    include/murdoch_rta/communicator/messages/BooleanMessage.h
    include/murdoch_rta/communicator/messages/BidMessage.h
    include/murdoch_rta/communicator/messages/AuctionMessage.h

    # Murdoch/Task
    include/murdoch_rta/task/Task.h

    # Murdoch/Resource
    include/murdoch_rta/resource/Resource.h
    include/murdoch_rta/resource/Discrete.h
    include/murdoch_rta/resource/Float.h
    include/murdoch_rta/resource/FloatRange.h
    include/murdoch_rta/resource/Integer.h
    include/murdoch_rta/resource/IntegerRange.h
    include/murdoch_rta/resource/Type.h

    # Murdoch/auctioneer
    include/murdoch_rta/auctioneer/Auctioneer.h
    include/murdoch_rta/auctioneer/State.h
    include/murdoch_rta/auctioneer/StateController.h

    # Murdoch/auctioneer/States
    include/murdoch_rta/auctioneer/states/StateType.h
    include/murdoch_rta/auctioneer/states/Initial.h
    include/murdoch_rta/auctioneer/states/AuctioningTask.h
    include/murdoch_rta/auctioneer/states/MonitoringTask.h
    include/murdoch_rta/auctioneer/states/ProcessingTask.h
    include/murdoch_rta/auctioneer/states/WaitingAcknowledgment.h

    # Murdoch/Bidder
    include/murdoch_rta/bidder/Bidder.h
    include/murdoch_rta/bidder/State.h
    include/murdoch_rta/bidder/StateController.h

    # Murdoch/Bidder/States
    include/murdoch_rta/bidder/states/StateType.h
    include/murdoch_rta/bidder/states/ExecutingTask.h
    include/murdoch_rta/bidder/states/ProcessingAuction.h
    include/murdoch_rta/bidder/states/RenewingTask.h
    include/murdoch_rta/bidder/states/WaitingResult.h

    # Nodes
    include/nodes/Node.h
    include/nodes/Auctioneer_node.h
    include/nodes/Bidder_node.h
    )

set(CPP_FILES
    #utils
    src/utils/task_monitoring/BaseTaskMonitor.cpp
    src/utils/task_generator/BaseTaskGenerator.cpp
    src/utils/robot/BaseRobot.cpp

    # Plugins
    src/murdoch_rta/UtilityCalculator.cpp
    src/murdoch_rta/BaseUtilityCalculator.cpp

    # Murdoch
    src/murdoch_rta/Robot.cpp

    # Murdoch/Communicator
    src/murdoch_rta/communicator/Listener.cpp
    src/murdoch_rta/communicator/Talker.cpp

    # Murdoch/Communicator/Messages
    src/murdoch_rta/communicator/messages/Message.cpp
    src/murdoch_rta/communicator/messages/BooleanMessage.cpp
    src/murdoch_rta/communicator/messages/BidMessage.cpp
    src/murdoch_rta/communicator/messages/AuctionMessage.cpp

    # Murdoch/Task
    src/murdoch_rta/task/Task.cpp

    # Murdoch/Resource
    src/murdoch_rta/resource/Resource.cpp
    src/murdoch_rta/resource/Discrete.cpp
    src/murdoch_rta/resource/Float.cpp
    src/murdoch_rta/resource/FloatRange.cpp
    src/murdoch_rta/resource/Integer.cpp
    src/murdoch_rta/resource/IntegerRange.cpp

    # Murdoch/auctioneer
    src/murdoch_rta/auctioneer/Auctioneer.cpp
    src/murdoch_rta/auctioneer/State.cpp
    src/murdoch_rta/auctioneer/StateController.cpp

    # Murdoch/auctioneer/States
    src/murdoch_rta/auctioneer/states/Initial.cpp
    src/murdoch_rta/auctioneer/states/AuctioningTask.cpp
    src/murdoch_rta/auctioneer/states/MonitoringTask.cpp
    src/murdoch_rta/auctioneer/states/ProcessingTask.cpp
    src/murdoch_rta/auctioneer/states/WaitingAcknowledgment.cpp

    # Murdoch/Bidder
    src/murdoch_rta/bidder/Bidder.cpp
    src/murdoch_rta/bidder/State.cpp
    src/murdoch_rta/bidder/StateController.cpp
    # Murdoch/Bidder/States
    src/murdoch_rta/bidder/states/ExecutingTask.cpp
    src/murdoch_rta/bidder/states/ProcessingAuction.cpp
    src/murdoch_rta/bidder/states/RenewingTask.cpp
    src/murdoch_rta/bidder/states/WaitingResult.cpp

    # Nodes
    src/nodes/Node.cpp
    src/nodes/Auctioneer_node.cpp
    src/nodes/Bidder_node.cpp
    )

## murdoch_rta library
add_library(${PROJECT_NAME} ${CPP_FILES})

add_dependencies(${PROJECT_NAME}
    ${Boost_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## auctioneer executable
add_executable(auctioneer_node
    src/Auctioneer.cpp
    ${CPP_FILES}
    )
add_dependencies(auctioneer_node
    ${Boost_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(auctioneer_node
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )

## Bidder executable
add_executable(bidder_node
    src/Bidder.cpp
    ${CPP_FILES}
    )
add_dependencies(bidder_node
    ${Boost_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(bidder_node
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
