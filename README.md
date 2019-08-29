# A solution for MAPC 2019

## Setup
1. Clone workspace: `git clone --recursive git@gitlab.tubit.tu-berlin.de:aaip-ss19/mapc_workspace.git`
2. [Setup `mapc_workspace`](https://gitlab.tubit.tu-berlin.de/aaip-ss19/mapc_workspace#clone-and-build)
3. Clone our repository into the /src directory: `git clone git@gitlab.tubit.tu-berlin.de:aaip-ss19/group2.git`
4. Run `catkin_make` in `mapc_workspace`

## Project Structure
The `strategy_1` folder contains the following sub-directories:

 - `launch`: Contains launch files necessary for strategy execution. We tested on `rhbp_agents_strategy_1.launch`
 - `src`: Contains submodules that define agent behavior, communication, sensors, tasks and mapping. 
 
## Launch

 1. In `mapc_workspace` run: `source devel/setup.bash` 
 2. To execute strategy: `roslaunch strategy_1 rhbp_agents_strategy_1.launch`
 3. There are different configs available in `configs` folder, you can run `TestConfigGroup2` or `SampleConfig2` for server.

