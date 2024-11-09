## Project insights
- I have been following a udemy course, to learn on the Robotic Operating system 2.
- With the level 3 of that course I learnt the following things:
-> How to create action servers and lifecycle nodes.
-> How to configure and transition between different states of lifecycle nodes.
-> Learnt a lot on goals, feedback and execution of a foal in action server.
-> How to create a launch file and launch the project with a single line of code.

- I have also made a final project "Turtle final project" which does the following things:
-> Creates a action server and a lifecycle node to configure the action server.
-> receives the goal for the movement of the turtle in turtlesim and also with its duration.
-> Kills the default turtle and replaces with a new spawned turtle for custom movements.
-> Configures and activates the action server, which then starts accepting the goal to move.
-> We can move the turtle by sending a goal to the action server or a command to /cmd_vel topic.

- It was such a great experience going through, to set up ubuntu  and ROS2 with the help of this udemy course: https://www.udemy.com/course/ros2-advanced-core-concepts/

- To verify my certificate on completion of this course, you can visit: https://www.udemy.com/certificate/UC-c4535bc5-d725-48ed-a5d6-8b3a0f9ca0bf/

## Execution of the project
- To run this project for yourself, create a ROS2 workspace,
- Compile it with colcon build, and go to the src directory.
- Then place these folders on the src directory, and again build the workspace.
- Now in the terminal, run
```bash
ros2 launch turtle_final_project_bringup turtle_final_project.launch.py
```

## Project structure
```
├── ros2_ws/
│    ├── include/
│    ├── build/
│    ├── log/
│    ├── src/
│        ├── actions_py/
│        ├── casper_interfaces/
│        ├── lifecycle_node_bringup/
│        ├── lifecycle_py/
│        ├── turtle_final_project/
│        └── turtle_final_project_bringup/
```
