"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM
from math import sin, cos, tan, sqrt

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(
        self,
        # body rate parameters
        kp_p=0.11, 
        kp_q=0.11, 
        kp_r=0.11,  
        # altitude parameters
        kp_z=35.0,
        kd_z=25.0,
        # Yaw parameter
        kp_yaw=0.9,
        # Roll Pitch parameters
        kp_roll=10.0,  
        kp_pitch=10.0, 
        # lateral parameters
        kp_x=0.22,
        kd_x=0.15,
        kp_y=0.22,
        kd_y=0.22,
    ):

        # Body rate parameters
        self.kp_r = kp_r
        self.kp_p = kp_p
        self.kp_q = kp_q   
        # altitude parameters
        self.kp_z = kp_z
        self.kd_z = kd_z
        # Yaw parameter
        self.kp_yaw = kp_yaw
        # Roll Pitch parameters
        self.kp_roll = kp_roll
        self.kp_pitch = kp_pitch
        # lateral parameters
        self.kp_x = kp_x
        self.kd_x = kd_x
        self.kp_y = kp_y
        self.kd_y = kd_y



    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory
        
        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds
            
        Returns: tuple (commanded position, commanded velocity, commanded yaw)
                
        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]
        
        
        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]
            
            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]
            
        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]
                
                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]
            
        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)
        
        
        return (position_cmd, velocity_cmd, yaw_cmd)
    
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command
            
        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        x_error = local_position_cmd[0] - local_position[0]
        x_error_dot = local_velocity_cmd[0] - local_velocity[0]
        #x_dot_dot_command = self.kp_x * x_error + self.kd_x * 
        p_term_x = self.kp_x * x_error
        d_term_x = self.kd_x * x_error_dot
        x_dot_dot_command = p_term_x + d_term_x + acceleration_ff[0]
        
        y_error = local_position_cmd[1] - local_position[1]
        y_error_dot = local_velocity_cmd[1] - local_velocity[1]
        p_term_y = self.kp_y * y_error
        d_term_y = self.kd_y * y_error_dot
        y_dot_dot_command = p_term_y + d_term_y + acceleration_ff[1]

        
        #print('LATERAL POSITION CONTROL: ', np.array([-x_dot_dot_command,-y_dot_dot_command]))
        return np.array([-x_dot_dot_command,-y_dot_dot_command])
    
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            acceleration_ff: feedforward acceleration command (+up)
            
        Returns: thrust command for the vehicle (+up)
        """
        z_error = altitude_cmd - altitude
        z_error_dot = vertical_velocity_cmd - vertical_velocity
        
        b_z = self.R(attitude)[2,2]
        
        p_term = self.kp_z * z_error
        d_term = self.kd_z * z_error_dot
        
        u_1_bar = p_term + d_term + acceleration_ff

        thrust = (u_1_bar - GRAVITY) / b_z

        #print('ALTITUDE CONTROL (thrust_cmd): ', min(thrust, MAX_THRUST))

        return min(thrust, MAX_THRUST)
        
    
    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame
        
        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll,pitch,yaw) in radians
            thrust_cmd: vehicle thruts command in Newton
            
        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        rotation_matrix = self.R(attitude)
        #c_d = thrust_cmd/DRONE_MASS_KG
       
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
            
        
        #print('ROLL PITCH CONTROL (p_c,q_c): ', rotation_rate)
        return np.array([rotation_rate[0],rotation_rate[1]]) 
    
    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame
        
        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            attitude: 3-element numpy array (p,q,r) in radians/second^2
            
        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        # A propertional controller on body rates to commanded moments
        # P, Q, R describes the angles of the body frame of the vehicle
        # P is the angular rotation rate of the X axis (body_rate[0])
        # Q is the angular rotation rate of the Y axis (body_rate[1])
        # R is the angular rotation rate of the Z axis (body_rate[2])
        # body_rate[0] = ROLL (P)
        # body_rate[1] = PICTH (Q)
        # body_rate[2] = YAW (R)
        
        p_error = body_rate_cmd[0] - body_rate[0]
        u_bar_p = self.kp_p * p_error
        q_error = body_rate_cmd[1] - body_rate[1]
        u_bar_q = self.kp_q * q_error
        r_error = body_rate_cmd[2] - body_rate[2]
        u_bar_r = self.kp_r * r_error
        
        u_cmd = np.array([u_bar_p,u_bar_q,u_bar_r])
        #print('BODY RATE CONTROL (u_cmd): ', u_cmd)
        return u_cmd 
        
    
    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate
        
        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians
        
        Returns: target yawrate in radians/sec
        """
        yaw_cmd = np.mod(yaw_cmd, 2.0*np.pi)
        yaw_error = yaw_cmd - yaw
        yaw_rate = self.kp_yaw * yaw_error
        #print('YAW CONTROL - OUTPUT YAW (r_c): ', yaw_rate)
        return yaw_rate  



    def R(self, attitude):
        roll,pitch,yaw = attitude
        r_x = np.array([[1, 0, 0],
                        [0, cos(roll), -sin(roll)],
                        [0, sin(roll), cos(roll)]])
    
        r_y = np.array([[cos(pitch), 0, sin(pitch)],
                        [0, 1, 0],
                        [-sin(pitch), 0, cos(pitch)]])
        
        r_z = np.array([[cos(yaw), -sin(yaw), 0],
                        [sin(yaw), cos(yaw), 0],
                        [0,0,1]])
        
        r = np.matmul(r_z,np.matmul(r_y,r_x))
        #print(r)
        
        return r
    
