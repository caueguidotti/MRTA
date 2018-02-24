# MURDOCH - MultiRobot Task Allocation (murdoch_rta)

## Introduction

This text aims to explain how to use the *murdoch_rta* package for ROS. I will briefly talk about the [MURDOCH architecture](#murdoch), describing which sort of problems it tackles. Later, I will explain the [code structure](#implementation-details) behind this package implementation, pointing out specially the [ROS nodes and topics](#nodes) implementation, state machines and main classes. Finally, I will guide you through on building a package using the *murdoch_rta* package.


## MURDOCH
MURDOCH is an auction based system for task allocation. According to the Gerkey taxonomy [[1]][1-ref], MURDOCH lies on single-task (ST) robots, single-robot (SR) tasks and tasks are featured as Instantaneous Assignment (IA). Thus, robots are restricted to work in one task at a time, tasks requires one robot to be executed, and MURDOCH does not enables users to have any future planning and tasks are instantaneously allocated. It is important to point out here that the single-robot task described may refer to a sub-task of a global task. Thus, although single-robot tasks may raise the doubt if this system is cooperative, one may see as cooperation the act of robots working separately yet simultaneously on small tasks in order to achieve a greater goal.

As in any auction, MURDOCH defines two distinct types of agents: Auctioneer and Bidder. Auctioneer is the agent that receives and auction a task, processing bids and evaluating the winner. Whilst the Bidder, if participating in an auction, bids on a task and waits for the result of that auction. In MURDOCH, the bidder also have the role to maintain a communication with a robot. This communication is necessary so the bidder may be able to evaluate its bid value and inform the robot that it should start or interrupt a task.

MURDOCH's auction is fairly simple as it is implemented as an one-round sealed-bid auction: each prospective buyer (bidder) submits a single bid, and the sale (task allocation) goes to the highest bidder. However, there are a few restriction that defines if a bidder may or may not participate in an auction. Each task has its required resources (such as battery level) and a bidder can only participate in an auction if it meets the necessary resources. Another restriction arises since MURODCH handles single-task robots, which is that a bidder cannot bid in a task if its associated robot is already executing a task.

The bid value or task fitness may be calculated in many different forms. As one can infer, the fitness for executing a task may be strongly dependent on the type of the task. For example, a task to explore or map a region may depend on the robots sensory capability, however, in a search and rescue task, although a robot may also be required to explore a region, the time to reach that region may be very important, and so the robot speed or maneuverability could be considered. In any case, the bid value represent how adequate a robot is for executing a certain task.

The above explanations are given to familiarize the reader regarding the MURDOCH system, providing only the most basics information. If the reader is interested or find the above explanations insufficient, I would indicate the in-depth study of Brian Gerkey [[2]][2-ref] which not only talks about MURDOCH, but also brings very interesting information about other systems.

## Implementation Details
### Nodes
Five main nodes are described here, those are the minimum nodes necessary for using this package: Auctioneer, Bidder, Watcher, and Task Generator.

#### Auctioneer
The following image illustrates the auctioneer node and related nodes and topics. As can be observed, two external nodes have to be defined to achieve a working architecture: a Watcher and a Task Generator. The latter node can take many forms as will be explained later. As it may raise doubts, external nodes here refers to nodes that are defined outside the main scope (namespace) of this package. They are defined as such since they are very dependent on the type of application.

Auctioneer listens to tasks, which are 'Task' messages (see murdoch_rta_msgs package), published in the '/tasks' topic. These tasks are generated and fed into the system by the Task Generator external node(s). They carry the information of the type of the task, its priority, its deadline and the required resources for executing such task. From analyzing such task, Auctioneer creates and publishes an 'Auction' message into the '/auctions' topic.

Auctioneer also listens to bids, which are 'Bid' messages, in the '/bids' topic. bids are published by Bidder node(s) and solely contain a reference to the bidding auction and the bid value.

Auctioneer and Bidder nodes communicate extra messages using the '/boolMsgs' topic. This topic was named as such since they are mainly status indicatives, such as 'Won Auction', 'Continue Task', 'Stop Task', 'Task Completed', etc, these messages are described in more details in the murdoch_rta_msgs package.

The other external node is the Watcher. This node is used to analyze the progress of a task. As can be inferred from the following image, Auctioneer uses a service as the form of communicating with this node of type 'GetTaskStatus'. Auctioneer makes a request of a task status, based on the task unique id (taskUID), the watcher nodes then responds if the task shall continue, stop or if it has been completed.

<p align="center">
  <a href='https://photos.google.com/share/AF1QipN5-S5Jll_Ba320kHJnhYFsARWfHE4fMPg565fdF8a4WwcpT0QUO0IxN17zB9silA?key=U3hVelJ1SDVtTEg1aFRkZDNiZVdJVnFJdjJHSlhB&source=ctrlq.org'><img src='https://lh3.googleusercontent.com/iKIAiCIy0CmiP1o0VwQanQ9co4iQSUURguY1efi48rYI5vq5PMsLsrhENky1Z_Eny2h3KKqKtDxXg_nZzMkpcVlDInFs71VyYyYT3TOS3kvKQ3aFaawt180PS2mfxKbeka_LTie4' /></a>
</p>

#### Bidder



[//]: # (Links References)
[1-ref]: http://robotics.stanford.edu/~gerkey/research/final_papers/mrta-taxonomy.pdf
[2-ref]: http://robotics.stanford.edu/~gerkey/research/final_papers/diss.pdf
