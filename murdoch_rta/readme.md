# MURDOCH - MultiRobot Task Allocation (murdoch_rta)

## Introduction

This text aims to explain how to use the *murdoch_rta* package for ROS. I will briefly talk about the [MURDOCH architecture](#murdoch), describing which sort of problems it tackles. Later, I will explain the [code structure](#implementation-details) behind this package implementation, pointing out specially the [ROS nodes and topics](#nodes) implementation, [state machines](#state-machines) and main classes. Finally, I will guide you through on building a package using the *murdoch_rta* package.


## MURDOCH
MURDOCH is an auction based system for task allocation. According to the Gerkey taxonomy [[1]][1-ref], MURDOCH lies on single-task (ST) robots, single-robot (SR) tasks and tasks are featured as Instantaneous Assignment (IA). Thus, robots are restricted to work in one task at a time, tasks requires one robot to be executed, and MURDOCH does not enables users to have any future planning and tasks are instantaneously allocated. It is important to point out here what the single-robot task described may refer to a sub-task of a global task. Thus, although single-robot tasks may raise the doubt if this system is cooperative, one may see as cooperation the act of robots working separately yet simultaneously on small tasks in order to achieve a greater goal.

As in any auction, MURDOCH defines two distinct types of agents: Auctioneer and Bidder. Auctioneer is the agent that receives and auction a task, processing bids and evaluating the winner. Whilst the Bidder, if participating in an auction, bids for a task and waits for the result of that auction. In MURDOCH, the bidder also has the role to maintain a communication with a robot. This communication is necessary so the bidder may be able to evaluate its bid value and inform the robot that it should start or interrupt a task.

MURDOCH's auction is fairly simple as it is implemented as an one-round sealed-bid auction: each prospective buyer (bidder) submits a single bid, and the sale (task allocation) goes to the highest bidder. However, there are a few restrictions that defines if a bidder may or may not participate in an auction. Each task has its required resources (such as battery level) and a bidder can only participate in an auction if it meets the necessary resources. Another restriction arises since MURODCH handles single-task robots, which is that a bidder cannot bid for a task if its associated robot is already executing a task.

The bid value or task fitness may be calculated in many different forms. As one can infer, the fitness for executing a task may be strongly dependent on the type of the task. For example, a task to explore or map a region may depend on the robots sensory capability, however, in a search and rescue task, although a robot may also be required to explore a region, the time to reach that region may be very important, and so the robot speed or maneuverability could be considered. In any case, the bid value indicates how adequate a robot is for executing a certain task.

The above explanations are given to familiarize the reader regarding the MURDOCH system, providing only the most basics information. If the reader is interested or find the above explanations insufficient, I would indicate the in-depth study of Brian Gerkey [[2]][2-ref] which not only talks about MURDOCH, but also brings very interesting information about other systems.

## Implementation Details
### Nodes
Two main nodes are described here, Auctioneer and Bidder. However, these nodes also communicate with other nodes: Watcher, Task Generator and the Robot. Some explanations of their objectives and communication are provided here.

#### Auctioneer
The following image illustrates the Auctioneer node and related nodes and topics. As can be observed, two external nodes have to be defined to achieve a working architecture: a Watcher and a Task Generator. The latter node can take many forms as will be explained later. As it may raise doubts, external nodes here refers to nodes that are defined outside the main scope (namespace) of this package.

Auctioneer listens to tasks, which are 'Task' messages (see murdoch_rta_msgs package), published in the '/tasks' topic. These tasks are generated and fed into the system by the Task Generator external node(s). They carry the information about the type of the task, its priority, its deadline and the required resources for executing such task. From analyzing such task, Auctioneer creates and publishes an 'Auction' message into the '/auctions' topic.

Auctioneer also listens to bids, which are 'Bid' messages, in the '/bids' topic. bids are published by Bidder node(s) and solely contain a reference to the bidding auction and the bid value.

Auctioneer and Bidder nodes communicate extra messages using the '/boolMsgs' topic. This topic was named as such since they are mainly status indicatives, such as 'Won Auction', 'Continue Task', 'Stop Task', 'Task Completed', etc, these messages are described in more details in the murdoch_rta_msgs package.

The other external node is the Watcher. This node is used to analyze the progress of a task. As can be inferred from the following image, Auctioneer uses a service as the form of communicating with this node of type 'GetTaskStatus'. Auctioneer makes a request for a task status, based on the task unique id (taskUID), the watcher nodes then responds if the task shall continue, stop or if it has been completed.

<p align="center">
  <img src="https://github.com/caueguidotti/MRTA/blob/master/imgs/AuctioneerNodeDiagram.png" alt="Auctioneer Node"</>
</p>


#### Bidder
As observed on the previous picture, the Bidder node(s) listens to the Auctioneer 'Auction' messages, and then places bids, using the 'Bid' message type, through the topic '/bids'. Moreover, it receives and sends 'BoolMsg' messages through the '/boolMsgs' topic. These messages inform if it won an auction, if it should continue or stop a task, etc, and the bidder also uses it to send acknowledges that it received such messages.

The following image illustrates the bidder node and other related nodes and topics. As can be observed, the bidder node also communicates with the acting agent in the system, the Robot. The Bidder will request information about the Robot resources before it starts bidding on an Auction. In addition, if the Bidder is winner on an auction it will send 'setTask' requests to the Robot which responds with a boolean indicating if the task was set.

<p align="center">
  <img src="https://github.com/caueguidotti/MRTA/blob/master/imgs/BidderRosNodeDiagram.png" alt="Bidder Node"</>
</p>

### State Machines
MURDOCH defines very clear behavior for each of its agents (Bidder and Auctioneer), and those behaviors are quite sequentially, for this reason, I chose to implement each agent algorithm using [state machines][3-ref].

#### Auctioneer
Auctioneer has five distinguish states: Initial, Processing Task, Auctioning Task, Waiting Acknowledgment and Monitoring Task. Each of these states are briefly described below.
* Initial: Upon the state machine creation, Initial state is called for starting a preprocessing stage of the state machine. Currently, its only role is to [transfer ownership][4-ref] of the task from the Auctioneer class to the State Machine Controller class.
* Processing Task: This state performs the auction message creation and publishes it.
* Auctioning Task: This state monitors incoming bids and when the auction should finish. At the ending of the auction, if any bid was received, I choose the highest bid and send to the respective bidder a 'WonAuction' 'BoolMsg'.
* Waiting Acknowledgment: The sole purpose of this state is to wait for a response from the auction winning bidder. This is done so the auctioneer is aware of a disconnection of the bidder and to act on it accordingly. A maximum timeout is used here to add some process or network delay tolerance.
* Monitoring Task: This is the state for contacting the Watcher node and to request the task progress. A timeout is also used here to wait for the Watcher response. A task is said to be renewed when the watcher responds with a 'BoolMsg' containing a 'TaskContinuing' message, otherwise ('TaskCancelled' or 'TaskCompleted') represents that the auctioneer monitoring behavior should cease.

This state machine is initiated for every task message input into the system, so the Auctioneer instantiates concurrent state machines.

The image bellow provides an illustration of the Auctioneer state machine, containing the states and the transitions.

<p align="center">
  <img src="https://github.com/caueguidotti/MRTA/blob/master/imgs/AuctioneerStateMachine.png" alt="Auctioneer State Machine"</>
</p>

#### Bidder
Bidder has four distinguish states: Processing Auction, Waiting Auction Result, Renewing Task, Executing Task. Each of these states are briefly described below.
* Processing Auction: This is the first state engaged when a new auction arrives. Its role is to check if the robot can participate in an auction and if so, also place a bid in that auction.
* Waiting Auction Result: This state waits for the end of an auction that the bidder participated. When the auction finishes it checks if the bidder won the auction and if so, the state machine transitions to the next state.
* Renewing Task: In this state, the bidder attempts to contact the Robot an allocates a task, if it is successful it will send an acknowledge message to the auctioneer informing that it can proceed to the Monitoring Task state.
* Executing Task: This is the state that the bidder waits to receive a 'Renew Task' message from the auctioneer. If it receives such message, it then goes back to the renewing task state, finishing the cycle.

Unlike the auctioneer state machine, the bidder only initializes one state machine, since the robots in Murdoch executes one task at a time.

The image bellow provides an illustration of the Bidder state machine, containing the states and the transitions.

<p align="center">
  <img src="https://github.com/caueguidotti/MRTA/blob/master/imgs/BidderStateMachine.png" alt="Bidder State Machine"</>
</p>

[//]: # (Links References)
[1-ref]: http://robotics.stanford.edu/~gerkey/research/final_papers/mrta-taxonomy.pdf
[2-ref]: http://robotics.stanford.edu/~gerkey/research/final_papers/diss.pdf
[3-ref]: https://en.wikipedia.org/wiki/State_pattern
[4-ref]: https://en.wikipedia.org/wiki/Smart_pointer

