# RVO using rospy

## About:
This repo consists of implementation of the paper [RVO]() in ROS using the ROS python linrary "rospy". **Turtlesim** was used as the simulator for it's simplicity to test out the concept proposed in the paper.

## Index:
1. Repo Contents
2. About RVO
3. Implementation results
4. How to use

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
(medium link)

---

## 3. Using this library on your system:
#### 1. Pre-requisites:
- ROS(developed in Lunar)
- Turtlesim
- Python 3

#### 2. Installing this package on your system:

#### 3. Startup ROS and Turtlesim simulator
After installing the package, fire-up the ROS core
```
$ roscore
```
Then, start turtlesim sumulator in another terminal
```
$ rosrun turtlesim turtlesim_node
```

#### 4. Run
Now in another terminal, run the python code `multi_process_agents.py`
```
$ ./multi_process_agents.py
```
You will now be prompted to type the number of agents. For example,
```
$ Type  # of agents: 3
```
This is what you will get*
(gif goes here)

 ---

 ## 4. Implementation Results:

 These are the results obtained for various number of agents.
