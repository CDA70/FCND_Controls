# WRITE-UP FCND CONTROLS

The aim of the project is to implement a low level flight controller for a quadcopter in python and further modified in C++. In the previous project, commanded positions were passes to the simulation, whereas in this project, commands will be passed as three directional body moments and thrust. Using position, velocity, attitude and body rates sent from the simulation.   

Both parts in the projects are using a simulator and this is the place where all code can be tested and validated.

As first we start with implementing a controller in python. The controller consists mainly out of 5 parts: 
* body rate controller
* altitude controller
* yaw controller
* roll and pitch controller
* lateral position controller




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


