# WRITE-UP FCND CONTROLS

> A note before I start: Programming the controller wasn't easy, and especially setting the parameters provided the necessary headaches and is more than time consuming. 

In this project a low level flight controller is implemented in python and further modified in C++. In the previous projects, commanded positions were passed to the simulation, whereas in this project, commands are passed as three directional body moments and thrust. 
Nested control loops are commanded by using a position, velocity, attitude and body rates.

## PYTHON intro
In python most of the code is written in `controller.py` where you find the most important methods in the `NonLinearController` class. The methods that we implemented are:
* body rate controller: a porportional controller on body rates.
* altitude controller: a controller that uses a down velocity to command thrust. 
* yaw controller: a linear / proportional heading controller to yaw rate commands.
* roll pitch controller: an attitude controller that uses local acceleration commands and outputs body rate commands.
* lateral positon controller: a linear position controller using local north east position and local north/east velocity.

The following images are provided by UDACITY and clearly explains the quadcopter controls architecture that we can use throughout the project.

![Control Structure](/images/control1.png)

The altitude controller breaks down as:
![Control Structure](/images/control2.png)


UDACITY provided a collection of python packages and the environment can be simply activated by running the command: `source activate fcnd`.  
The simulator works in the same way as in the previous projects and the start screen looks as:
![Controls Simulator](/images/python-simulator.png)


## C++ intro
> All implemented code can be found in QuadControl.cpp. 

In my opinion, it was a lot more difficult and definitely not as straight forward after the python code. Especially tuning the  parameters is very cumbersome and something I struggled with. The parameters can be found in the `config/QuadControlParams.txt` file. 

For C++ a different simulation with more real limits is used. Each scenario in the simulation runs in a loop and each loop ends in a PASS or FAIL result and are presented at the end of the write-up 

## 1. Implemented body rate control in python and C++.
> The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

### python
Tuning parameters: 
`kp_p` 
`kp_q` 
`kp_r`

The commanded roll, pitch, and yaw are collected by the body rate controller and translated in rotational accelerations along the axis in the body frame

![equations body rate](/images/python-equations-body-rate.png)

``` python
    p_error = body_rate_cmd[0] - body_rate[0]
    u_bar_p = self.kp_p * p_error
    q_error = body_rate_cmd[1] - body_rate[1]
    u_bar_q = self.kp_q * q_error
    r_error = body_rate_cmd[2] - body_rate[2]
    u_bar_r = self.kp_r * r_error
        
    u_cmd = np.array([u_bar_p,u_bar_q,u_bar_r])
```

### C++
Tuning parameters:
`kpPQR`

Without the `kpPQR`, the drone keeps flipping. It's hard to find a good value, but I hope I managed to tune the parameter. The `Ixx Iyy Izz` are part of the `BaseController` and defining the mass moment of inertia.

The commanded thrust is the mass moment of the inertia.

``` C++
V3F momentCmd;
V3F Inertia;
V3F p_error = pqrCmd - pqr;
    
float Inertia.x = Ixx;
float Inertia.y = Iyy;
float Inertia.z = Izz;
   
momentCmd = Inertia * p_error * kpPQR;
```


## Implement roll pitch control in python and C++.
> The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

The roll-pitch controller is a P controller responsible for commanding the roll and pitch rates ( `pc` and `qc` ) in the body frame. It sets the desired rate of change of the given matrix elements using a P controller.
### python
Tuning parameters:
`kp_roll`, 
`kp_pitch`

![equations roll pitch](/images/python-roll-pitch1.png)

where `b_x_a = R13` and `b_y_a = R23.` The values of R13 and R23 can be converted into angular velocities into the body frame by a matrix multiplication:

![matrix multiplication](/images/python-matrix-multiplication.png)


``` python

    # R13
    b_x = rotation_matrix[0,2]
    b_x_error = (acceleration_cmd[0] - b_x)  
    b_x_commanded_dot = self.kp_roll * b_x_error 
        
    # R23
    b_y = rotation_matrix[1,2]
    b_y_error = (acceleration_cmd[1] - b_y) 
    b_y_commanded_dot = self.kp_pitch * b_y_error 
        
    rotation_matrix_update = np.array([[rotation_matrix[1,0], -rotation_matrix[0,0]],
                                      [rotation_matrix[1,1], -rotation_matrix[0,1]]]) / rotation_matrix[2,2]
        
    rotation_rate = np.matmul(rotation_matrix_update,
                            np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
```

The rotation matrix is as follows and be implement as: 

![rotation matrix](/images/python-rotation-matrix.png)

``` python
    r_x = np.array([[1, 0, 0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll), cos(roll)]])
    
    r_y = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])
        
    r_z = np.array([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw), cos(yaw), 0],
                    [0,0,1]])
```


### C++
Tuning Parameters:
`kpBank`

The parameter `kpBank` and `kpPQR` shall be tuned, but as I mentioned earlier, it's hard and it took sooooo and tooooo much time
.
The collThrustCmd is a force in Newton and must be converted in acceleration by dividing it by MASS. Keep in mind the downwards effect, hence the `-` negative in front of collThrustCmd

``` c++
    float b_x = R(0,2);
    float b_x_error = (accelCmd.x / (- collThrustCmd / mass)) - b_x;
    float b_x_commanded_dot = kpBank * b_x_error;
    
    
    float b_y = R(1,2);
    float b_y_error = (accelCmd.y / (- collThrustCmd / mass)) - b_y;
    float b_y_commanded_dot = kpBank * b_y_error;
    
    //// rollPitchCmd.x = (R21 * bxCommandedDot - R11 * byCommandedDot) / R33
    //// rollPitchCmd.y = (R22 * bxCommandedDot - R12 * byCommandedDot) / R33
    //// rollPitchCmd.z = 0.0;
    pqrCmd.x = (R(1,0) * b_x_commanded_dot - R(1,1) * b_y_commanded_dot) / R(2,2);
    pqrCmd.y = (R(1,1) * b_x_commanded_dot - R(0,1) * b_y_commanded_dot) / R(2,2);
    pqrCmd.z = 0.0;
```


## Implement altitude control in python.
> The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

> Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

The altitude controller is a PD controller to control acceleration and can be expressed by the following linear equation:

![equations altitude](/images/altitude-equation.png)

where R equals to: 

![altitude R](/images/altitude-r.png)

in more details: 
`x_dot_dot = cb_x`

`y_dot_dot = cb_y`

`z_dot_dot = cb_z + g`

where `b_x = R13` , `b_y = R23` , `b_z = R33`, which are all elements of the Rotation matrix.

The vertical acceleration is controlled by:

![altitude vertical acceleration](/images/altitude-vertical-acceleration.png)

The PD controller outputs the u_bar and is seen as:

![altitude u_bar](/images/altitude-u-bar.png)

`u_bar = kp_z(z_t - z_a) + kd_z(z_dot_t - z_dot_a) + z_dot_dot_t`

### python
Tuning parameters:
`kp_z`, 
`kd_z`

``` python
    z_error = altitude_cmd - altitude
    z_error_dot = vertical_velocity_cmd - vertical_velocity
        
    b_z = self.R(attitude)[2,2]
        
    p_term = self.kp_z * z_error
    d_term = self.kd_z * z_error_dot
        
    u_1_bar = p_term + d_term + acceleration_ff
```
keep in mind the GRAVITY that will influence the thrust:  `(u_1_bar - GRAVITY) / b_z`

### C++
Tuning parameters:
`kpPosZ`,  
`kpVelZ`, 
`KiPosZ`

We added the integral control to help with the different masses of the vehicle as despicted in scenario 4.
The CONSTRAIN is very useful and replaces my previous IF control structure (many thanks to UDACITY for the very good tip).
``` C++
    float z_error = posZCmd - posZ;
    float z_error_dot = velZCmd - velZ;
    float b_z = R(2,2);
    
    float p_term = kpPosZ * z_error;
    float d_term = kpVelZ * z_error_dot;
    
    integratedAltitudeError = integratedAltitudeError + (z_error * dt);
    float i_term = KiPosZ * integratedAltitudeError;
    
    float u_1_bar = p_term + d_term + accelZCmd + i_term;
    
    float acceleration = (u_1_bar - CONST_GRAVITY) / b_z;
    
    thrust = - mass * CONSTRAIN(acceleration, - maxAscentRate / dt, maxAscentRate / dt);
```

## Implement lateral position control in python and C++.
> The controller should use the local NE position and velocity to generate a commanded local acceleration.

Like the altitude controller, the lateral position controller is a PD controller to command target values for elements of the drone's rotation matrix. It generates lateral acceleration by changing the body orientation which results in non-zero in either `x` and `y` direction. The follwing equation shows the commanded rotation matrix elements `b_x_c`, but the same applies for `b_y_c`

![lateral controller](/images/lateral-controller1.PNG)

### python
Tuning parameters:
`kp_x`, 
`kd_x`, 
`kp_y`, 
`kd_y`

``` python
    x_error = local_position_cmd[0] - local_position[0]
    x_error_dot = local_velocity_cmd[0] - local_velocity[0]
    p_term_x = self.kp_x * x_error
    d_term_x = self.kd_x * x_error_dot
    x_dot_dot_command = p_term_x + d_term_x + acceleration_ff[0]
        
    y_error = local_position_cmd[1] - local_position[1]
    y_error_dot = local_velocity_cmd[1] - local_velocity[1]
    p_term_y = self.kp_y * y_error
    d_term_y = self.kd_y * y_error_dot
    y_dot_dot_command = p_term_y + d_term_y + acceleration_ff[1]
```


### C++
Tuning parameters:
`kpPosXY`, 
`kpVelXY`


``` C++
    velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
    velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
    
    V3F position_error = posCmd - pos;
    V3F velocity_error = velCmd - vel;
    
    V3F pos_term = kpPosXY * position_error;
    V3F vel_term = kpVelXY * velocity_error;
    
    accelCmd = pos_term + vel_term + accelCmd;
    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```
The function `CONSTRAIN` is used to limit to range of `maxSpeedXY` and `maxAccelXY `. 

## Implement yaw control in python and C++.
> The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

Unlike the altitude and the lateral controller, the yaw is a P controller and the equation is fairly simple to implement.

![yaw controller](/images/yaw-controller1.PNG)

### python
Tuning parameters: 
`kp_yaw`

``` python
    yaw_cmd = np.mod(yaw_cmd, 2.0*np.pi)
    yaw_error = yaw_cmd - yaw
    yaw_rate = self.kp_yaw * yaw_error
```

### C++
Tuning parameters:
`kpYaw`

``` C++
    float yaw_error = yawCmd - yaw;
    yawRateCmd = kpYaw * yaw_error;
```

## Implement calculating the motor commands given commanded thrust and moments in C++.
> The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

The method `GenerateMotorCommands` is the code where I started with. The method converts a desired 3-axis moment and collective thrust command to individual motor thrust commands. 

The result must be a total force:
* total force: `F_total = F0 + F1 + F2 + F3`
* roll:        `tau_x = (F0 - F1 + F2 - F2) * l`
* pitch:       `tau_y = (F0 + F1 - F2 - F3) * l`
* yaw:         `tau_z = (-F0 + F1 + F2 - F3) * kappa`

`l` is the length and is defined as half the distance between the rotors
`l = L / sqrt(2)`

Input parameters  `collThrustCmd, momentCmd` with the desired rotation moment is given and it provides the `F_total, tau_x, tau_y, tau_z`. 
with these values we can further calculate the thrust command.

``` C++
    float length = L / sqrt(2.f);
    float thrust_x = momentCmd.x / length;
    float thrust_y = momentCmd.y / length;
    float thrust_z = -momentCmd.z / kappa; // kappa = torque (Nm) produced by motor per N of thrust produced
    
    cmd.desiredThrustsN[0] = ( thrust_x + thrust_y + thrust_z + collThrustCmd) / 4.f; //front left
    cmd.desiredThrustsN[1] = (-thrust_x + thrust_y - thrust_z + collThrustCmd) / 4.f; //front right
    cmd.desiredThrustsN[2] = ( thrust_x - thrust_y - thrust_z + collThrustCmd) / 4.f; // rear left
    cmd.desiredThrustsN[3] = (-thrust_x - thrust_y + thrust_z + collThrustCmd) / 4.f; //rear right
```


# EVALUATION:
## Your python controller is successfully able to fly the provided test trajectory, meeting the minimum flight performance metrics.
> For this, your drone must pass the provided evaluation script with the default parameters. These metrics being, your drone flies the test trajectory faster than 20 seconds, the maximum horizontal error is less than 2 meters, and the maximum vertical error is less than 1 meter.

The drone can flies the test trajectory by executing `python controls_flyer.py`. The terminal output shows a successful mission: 

![result](/images/python-result-controls-flyer.png)

the trajectory can be displayed in a 2D chart representation by using [`VisDom`](https://github.com/facebookresearch/visdom)

![2D charts](/images/python-visdom-2D-chart.png)

I used GIPHY to capture a gif from executing the Python code, but apparently GIPHY only captures 30 sec. Most of the flight is capture, especially most of the waypoint phase is recorded. The arming, takeoff, landing disarming are not in the gif included.

![python flight](/images/python-controls-flyer.gif)

## Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
> Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).

It wasn't easy to handle the `tuning parameters`. I'm sure that the result can be improved, but I have a hard time understanding the relationship between the code debugging and tuning of parameters.

### Scenario 1 - Intro:
In scenario 1, the thrust was simply `cmd.desiredThrustsN[0] = mass * 9.81f / 4` where mass was equal to 0.4 kg and made the quadcopter falling down. Increasing the mass to __0.5 kg__ ensured a proper hover of the quadcopter.

![C++ Scenario 1](/images/cpp-Scenario1.gif)

PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds

### Scenario 2 - attitude control:
In scenario 2, roll and pitch is tested. The requirement is to stabalize the rotational motion of the controller.

![C++ Scenario 3](/images/cpp-scenario2.gif)

PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

### Scenario 3 - position control:
In scenario 3 the position, altitude and yaw are tested. Two identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

![C++ Scenario 3](/images/cpp-scenario3.gif)

PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

### Scenario 4 - Non Idealities
In this scenario, the robustness of the controller is tested. Three quads are all trying to move one meter forward. However, this time, these quads are all a bit different:

* The green quad has its center of mass shifted back
* The orange vehicle is an ideal quad
* The red vehicle is heavier than usual

![C++ Scenario 4](/images/cpp-scenario4.gif)

PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds

### Scenario 5 - Follow traject
In scenario 5 the quad tries to follow a given trajectory. Unfortunately tuning the control parameters is very time consuming and I'm running out of time. I will further optimize the result, but this is result of the provided code.

![C++ Scenario 5](/images/cpp-scenario5.gif)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds

## next is a screenshot with the terminal output of the CPP results

![C++ results](/images/cpp-results-all-scenarios.png)

The project wasn't easy and provided lots of challenges, especially tuning the parameters was difficult. Also it's not easy to debug the program because the expected result of each variable isn't clear and requires the necessary experience. 
At the end it was fun and I'm sure that the next lessons will provide some answers to better understand and tune the parameters.