A robot base which can autonomously navigate with obstacle avoidance capability has been programmed to be used as a butler in a hotel environment with the ability to handle multiple orders with real-time acknowledgements and order handling capacity.

# Butler Controller

## Overview
The Butler Controller is a ROS-based autonomous system designed to handle food delivery tasks in a simulated environment. It controls a robot that moves between predefined locations (kitchen, tables, and home) while following a structured workflow based on user inputs.

# Simulated Hotel Environment:
<img width="425" alt="Top-view-of-a-simulated-environment-of-a-house-in-Gazebo" src="https://github.com/user-attachments/assets/dc64934d-198f-480c-b45d-986ea78cf923" />

![rviz_map](https://github.com/user-attachments/assets/5633291d-4cdf-4f31-b3df-54af4f01dd48)

## Features
- Receives orders from tables and processes them accordingly.
- Moves to the kitchen to pick up orders.
- Waits for kitchen confirmation or times out and returns home.
- Delivers orders to the respective tables.
- Waits for table confirmation before moving to the next location.
- Cancels orders if requested and returns to the kitchen before going home.
- Autonomous navigation integrated with RViz and Move Base.

---

## Sequence of Operation
1. **Launch Simulation**: Start the necessary files to bring up the robot in the simulation world along with RViz and autonomous navigation.
2. **Run Butler Controller**: Execute the Butler Controller script to start the task manager.
4. **Publish Orders**: At least one order must be published to initiate the delivery process.
5. **Kitchen Confirmation**: The robot will wait for confirmation from the kitchen. If no confirmation is received within the timeout period, it will return home.
6. **Table Confirmation**: The robot will wait for confirmation from the respective table before marking the order as delivered.
7. **Return Home**: Once delivery is complete, the robot will automatically return home.
8. **Cancel Orders**: If an order is canceled, the robot will go back to the kitchen and then return home.

---

## Dependencies
- ROS (Robot Operating System)
- `geometry_msgs`
- `std_msgs`
- `actionlib_msgs`
- `butler` (custom message package for TableStatus)
- Move Base for navigation

---

## ROS Topics Used
### Published Topics
- `/move_base_simple/goal` (`PoseStamped`): Sends navigation goals to the robot.
- `/move_base/cancel` (`GoalStatusArray`): Cancels current navigation goals.

### Subscribed Topics
- `/move_base/status` (`GoalStatusArray`): Monitors the robot's navigation status.
- `/table_confirm` (`Bool`): Receives table confirmation after order delivery.
- `/kitchen_confirm` (`Bool`): Receives kitchen confirmation for order pickup.
- `/table1`, `/table2`, `/table3` (`TableStatus`): Receives order placement or cancellation updates.

---

## Workflow Explanation
1. When an order is placed at any table, the robot moves to the kitchen.
2. The robot waits for kitchen confirmation. If confirmed, it proceeds to deliver the order; otherwise, it returns home after a timeout.
3. The robot moves to the respective table and waits for table confirmation.
4. After confirmation, it returns home.
5. If an order is placed and later canceled, the robot moves back to the kitchen and then returns home.

---

## Running the Butler Controller
# Clone this repository in your ROS workspace:
```
  cd ~/caktin_ws/src
  git clone https://github.com/Thinesh-Kumar-T-M/autonomous_butler_bot
```
Now
```
  cd ~/catkin_ws && catkin_make

```
To start the controller, execute the following command:
```
rosrun butler butler_controller.py
```
## Make sure the simulation and navigation stack are already running:

  Refer this repository for simulation setup: <link>https://github.com/Thinesh-Kumar-T-M/Autonomous-Mobile-Robot-Cream_Pi?tab=readme-ov-file#autonomous-mobile-robot-cream_pi</link>
---

## Testing the Controller
To manually test the system, use the following ROS commands:

### 1. Placing an Order
```
rostopic pub /table1 butler/TableStatus '{order: true, cancel: false, table_id: "table1"}'
rostopic pub /table2 butler/TableStatus '{order: true, cancel: false, table_id: "table2"}'
rostopic pub /table3 butler/TableStatus '{order: true, cancel: false, table_id: "table3"}'
```

### 2. Kitchen Confirmation
```
rostopic pub /kitchen_confirm std_msgs/Bool data:true
```

### 3. Table Confirmation
```
rostopic pub /table_confirm std_msgs/Bool data:true
```

### 4. Canceling an Order
```
rostopic pub /table1 butler/TableStatus '{order: true, cancel: true, table_id: "table1"}'
rostopic pub /table2 butler/TableStatus '{order: true, cancel: true, table_id: "table2"}'
rostopic pub /table3 butler/TableStatus '{order: true, cancel: true, table_id: "table3"}'
```
