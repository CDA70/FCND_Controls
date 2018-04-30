# WRITE-UP FCND CONTROLS

> A note before I start: Programming the controller wasn't easy, and especially setting the parameters provided the necessary headaches and is more than time consuming. 

In this project a low level flight controller is implemented in python and further modified in C++. In the previous projects, commanded positions were passed to the simulation, whereas in this project, commands will be passed as three directional body moments and thrust. 
Nested control loops are commanded by using a position, velocity, attitude and body rates.

## PYTHON intro
In python most of the code is written in `controller.py` where you find the most important methods in the `NonLinearController` class. The methods that we implemented are:
* body rate controller: a porportional controller on body rates.
* altitude controller: a controller that uses a down and down velocity to command thrust. 
* yaw controller: a linear / proportional heading controller to yaw rate commands.
* roll pitch controller: an attitude controller that uses local acceleration commands and outputs body rate commands.
* lateral positon controller: a linear position controller using local north east position and local north/east velocity.

![Control Structure](/images/control1.png)

The altitude controller breaks down as:
![Control Structure](/images/control2.png)


Lots of the underlying python code was provided by UDACITY as a collection of python package. Run the following command to activate the collection or environment: `source activate fcnd`.  
The simulator works in the same way as in the previous projects and the start screen looks as:
![Controls Simulator](/images/python-simulator.png)

running the code in python `python controls_flyer.py` result into the following:

### terminal output
![result](/images/python-result-controls-flyer.png)


## C++ intro
> All implemented code can be found in QuadControl.cpp. 

In my opinion, it was a lot more difficult and definitely not as straight forward after the python code. Especially the parameters is a work that I struggle with. The parameters can be found in the `config/QuadControlParams.txt` file. 

For C++ a different simulation with more real limits is used. Each scenario in the simulation runs in a loop and each loop ends in a PASS or FAIL result. 

## 1. Implemented body rate control in python and C++.
> The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

### python
parameters: 
`kp_p=0.11` 
`kp_q=0.11` 
`kp_r=0.11`

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
parameters:
`kpPQR =  80,80,5`

Without the `kpPQR`, the drone keeps flipping. it's hard to find a good value, but i belive I managed to tune the parameter. The `Ixx Iyy Izz` are part of the `BaseController` and defining the mass moment of inertia.

The commanded thrust is the mass moment of the inertia `*` the error of the desired body rates `*`` the kpPQR parameter

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
![equations roll pitch](/images/python-roll-pitch1.png)
where `b_x_a = R13` and `b_y_a = R23.` The values of R13 and R23 can be converted into angular velocities into the body frame by a matrix multiplication:

![matrix multiplication](/images/python-matrix-multiplication.png)

``` python
    # R13
    b_x = rotation_matrix[0,2]
    b_x_error = (acceleration_cmd[0] - b_x) 
    b_x_commanded_dot = self.kp_roll * b_x_error #/ c_d
        
    # R23
    b_y = rotation_matrix[1,2]
    b_y_error = (acceleration_cmd[1] - b_y) 
    b_y_commanded_dot = self.kp_pitch * b_y_error #/ c_d
        
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
Parameters:
`kpBank`

The parameter `kpBank` and `kpPQR` shall be tuned, but as I mentioned earlier, it's hard and has taken me endless time, I still try to understand the result.
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
u_bar = kp_z(z_t - z_a) + kd_z(z_dot_t - z_dot_a) + z_dot_dot_t

### python
parameters:
`kp_z`
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
parameters:
`kpPosZ` 
`kpVelZ`
`KiPosZ`

We added the integral control to help with the different masses of the vehicle as despicted in scenario 4.
The CONSTRAIN is very useful and replaces the previous IF control structure.
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
The controller should use the local NE position and velocity to generate a commanded local acceleration.

## Implement yaw control in python and C++.
The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

## Implement calculating the motor commands given commanded thrust and moments in C++.
> The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

The method `GenerateMotorCommands` is the first modification I have done. The method converts a desired 3-axis moment and collective thrust command to individual motor thrust commands. 
![result](/images/cpp-motors-thrusts.png)

# EVALUATION:
## Your python controller is successfully able to fly the provided test trajectory, meeting the minimum flight performance metrics.
For this, your drone must pass the provided evaluation script with the default parameters. These metrics being, your drone flies the test trajectory faster than 20 seconds, the maximum horizontal error is less than 2 meters, and the maximum vertical error is less than 1 meter.

## Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.
Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).


