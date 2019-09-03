# A solution for MAPC 2019

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

