# RVO using rospy

## About:
This repo consists of implementation of the paper [RVO]() in ROS using the ROS python linrary "rospy". **Turtlesim** was used as the simulator for it's simplicity to test out the concept proposed in the paper.

## Index:
1. Repo Contents
2. About RVO
3. Implementation Results

---

## 1. Repo Contents:
- [`turtle_instance.py`]() consists of a class that describes the behaviour of each agent.
- [multi_process_agents.py]() consists of the code that:
  - generates the specified number of agents and
  - Each agent then creates an object from class defined in "turtle_instance.py"
  - Also, it assigns an individual process on CPU for each of the generated agent.
- [Individual/]() consists of code where each python file is responsible for each agent. [This is a previous version before writing multi_process_agents.py]
- [repo_support_files/]() support files for README.md file.

---

## 2. About RVO:

[Paper Review: Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation](https://medium.com/@suraj2596/paper-review-reciprocal-velocity-obstacles-for-real-time-multi-agent-navigation-aaf6adbedefd)

---

## 3. Implementation Results:

These are the results obtained for various number of agents.
- 2-agent: https://www.youtube.com/watch?v=wt4jghNB_5w
- 4-agent: https://www.youtube.com/watch?v=gfRwZAqging
- 12-agent: https://www.youtube.com/watch?v=ChxYs4OLPwg
