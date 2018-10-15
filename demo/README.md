# ROS Packages for Neurobots Demonstrator


## Packages:
  
database_conversions(Provides Knowledge Base (KB) representation of the PDDL World + Conversions of KB to ROS Messages)

database_msgs (Specifies messages and actions for sharing and exchanging world knowledge)

neurobots_database (Implements World Knowledge Base and Communication Server)

neurobots_launch (Launch files to start Knowledge Base Server)


## Scenario Definition in folder:

scenario
- also read the [Wiki Entry](http://mlgitlab.informatik.uni-freiburg.de/neurobots/demo/wikis/home) for a description

## Installation Instructions

To install a new ROS workspace that enables to run the demo, please [read this guide](installation_guide.md).

## Starting the Demo

To start the demo you need to launch the following launch files:

- Robots: roslaunch neurobots_launch hardware.launch
- Demo: roslaunch neurobots_launch demo.launch