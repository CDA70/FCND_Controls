# WRITE-UP FCND CONTROLS
In this project a low level flight controller is implemented in python and further modfied in C++. In the previous projects, commanded positions were passed to the simulation, whereas in this project, commands will be passed as three directional body moments and thrust. 
Nested control loops are commanded by using a position, velocity, attitude and body rates.

In python most of the code is written in `controller.py` where you find the most important methods in the `NonLinearController` class. The methods that we implemented are:
* body rate controller: a porportional controller on body rates.
* altitude controller: a controller that uses a down and down velocity to command thrust. 
* yaw controller: a linear / proportional heading controller to yaw rate commands.
* roll pitch controller: an attitude controller that uses local acceleration commands and outputs body rate commands.
* lateral positon controller: a linear position controller using local north east position and local north/east velocity

![Control Structure](https://github.com/CDA70/FCND_Controls/images/ControlStructure.png)

## 1. Implemented body rate control in python and C++.
The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

## Implement roll pitch control in python and C++.
The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

## Implement altitude control in python.
The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

## Implement altitude controller in C++.
The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

## Implement lateral position control in python and C++.
The controller should use the local NE position and velocity to generate a commanded local acceleration.

## Implement yaw control in python and C++.
The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

## Implement calculating the motor commands given commanded thrust and moments in C++.
The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

# EVALUATION:
## Your python controller is successfully able to fly the provided test trajectory, meeting the minimum flight performance metrics.
For this, your drone must pass the provided evaluation script with the default parameters. These metrics being, your drone flies the test trajectory faster than 20 seconds, the maximum horizontal error is less than 2 meters, and the maximum vertical error is less than 1 meter.

## Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).


