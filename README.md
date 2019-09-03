# A solution for MAPC 2019

_Our team's solution to the MAPC 2019 makes use of ROS and RHBP to have multiple agents work on a strategy to submit as many tasks as possible. The agents communicate with each other and use their perception to merge their local maps with those of other agents to get a global view of the map. The hybrid planner helps agents to take the best suited action to complete this strategy. Agents are able to do path planning by making use the A-star algorithm. *Our task submission strategy makes the assumption that all tasks have only a one block submission requirement.*_

## Setup

1. Clone the repository
2. Run `catkin_make` 

## Project Structure
The folder contains the following sub-directories:

 - `launch`: Contains launch files necessary for strategy execution. We tested on `rhbp_agents_strategy_1.launch`
 - `src`: Contains submodules that define agent behavior, communication, sensors, tasks and mapping. 
 
## Launch

 1. In the main workspace run: `source devel/setup.bash` 
 2. To execute strategy: `roslaunch strategy_1 rhbp_agents_strategy_1.launch`
 3. There are different configs available in `configs` folder, you can run `TestConfigGroup2` or `SampleConfig2` for server.


## Strategy

![](./figures/behaviours_sensors.png)

**<p align="center">Behaviours and Sensors</p>**


