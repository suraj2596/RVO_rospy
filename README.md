# RVO using rospy
![](https://cdn-images-1.medium.com/max/600/1*-PEHaKXuBFPBC0HdQ-Kjjg.gif)

## About:
This repo consists of implementation of the paper [RVO]() in ROS using the ROS python linrary "rospy". **Turtlesim** was used as the simulator for it's simplicity to test out the concept proposed in the paper.

## Index:
1. Repo Contents
2. About RVO
3. Using this repo
4. Implementation Results

---

## 1. Repo Contents:
- `src/turtle_instance.py` consists of a attributes and methods that describe the behaviour of each agent.
- `src/multi_process_agents.py` consists of the code that:
  - generates the specified number of agents and
  - Each agent then creates an object from class defined in `turtle_instance.py`
  - Also, it assigns an individual process on CPU for each of the generated agent.
- `src/Individual/` consists of code where each python file is responsible for each agent. (This is a previous version before writing multi_process_agents.py)

---

## 2. About RVO:

[Paper Review: Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation](https://medium.com/@suraj2596/paper-review-reciprocal-velocity-obstacles-for-real-time-multi-agent-navigation-aaf6adbedefd)

---

## 3. Using this repo
1. Clone this package into your catkin workspace

`git clone https://github.com/suraj2596/RVO_rospy/`

2. Do a catkin make at the root of your workspace

`catkin_make`

3. Start ROS

`roscore`

4. In another terminal, start turtlesim

`rosrun turtlesim turtlesim_node`

5. In another terminal, run this command to spawn a turtle with name 'turtlex'

`rosservice call /spawn "x: 0.0 y: 0.0 theta: 0.0 name: ''"`

6. In another terminal, run multi_process_agents.py

`python multi_process_agents.py`


---
## 4. Implementation Results:

These are the results obtained for various number of agents.
- 2-agent: https://www.youtube.com/watch?v=wt4jghNB_5w
- 4-agent: https://www.youtube.com/watch?v=gfRwZAqging
- 12-agent: https://www.youtube.com/watch?v=ChxYs4OLPwg
