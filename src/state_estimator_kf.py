#!/usr/bin/env python3
import rospy
import math
import time

import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_matrix

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from rosgraph_msgs.msg import Clock

from nav_msgs.msg import Odometry

####################### Global variables declarations #######################

obtained_initial_orientation_flag = False
max_velocity = 33.33 # m/s
fs =100 # Hz
T = 1./fs # s
L = 2.269 # m

'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = A * X + B * U
    2] Predict Error Covariance: P_hat = A * P * transpose(A) + Q
    3] Calculate Kalman gain: K = P * transpose(C) * inverse(C * P * transpose(C) + R)
    4] Update State: X = X_hat + K * [Z - C * X_hat]
    5] Update Error Covariance: P = [I - K * C] * P_hat

    Now it is important to note that:

    1] X_matrix: The states of the system and they are transpose([x, y, vx, vy yaw, yaw_dot, steering angle])
    2] X_hat_matrix: These are the initially estimated states
    3] A_matrix: The state transition matrix
    4] B_matrix: The input mapping matrix [vy, yaw_dot, steering angle]
    5] U_matrix: A matrix containing the inputs to the system
    6] C_matrix: This is the matrix mapping states to outputs
    7] P_matrix: This is the prediction error Covariance matrix
    8] P_hat_matrix: This is the initially estiamted prediction error Covariance matrix
    9] Q_matrix: This is the process noise Covariance matrix. It is assumed to be white gaussian noise
    10] R_matrix: This is the sensor measurement error Covariance matrix. This one was computed via a script analyzing stationary readings
    11] Z_matrix: This is a matrix containing the current sensors' readings
    12] K_matrix: This is the Kalman gains matrix
    13] I_matrix: An Identity Matrix

      Kalman Parameters initialization:
'''

# [vx, vy, vz, ax, ay, az, yaw, Yaw_dot, steering angle]T
X_matrix = np.zeros((9,1))

X_hat_matrix = np.zeros((9,1))
                #     x  y Vx  Vy h  h' s  a
A_matrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0], # x
                     [0, 1, 0, T, 0, 0, 0, 0, 0], # y
                     [0, 0, 0, 0, 0, 0, 0, 0, 0], # Vx
                     [0, 0, 0, 1, 0, 0, 0, T, 0], # Vy
                     [0, 0, 0, 0, 1, T, 0, 0, 0], # heading
                     [0, 0, 0, 0, 0, 1, 0, 0, T], # heading rate
                     [0, 0, 0, 0, 0, 0, 0, 0, 0], # Steering angle
                     [0, 0, 0, -1.5, 0, 0, 0, 0, 0], # a
                     [0, 0, 0, 0, 0, -1.35, 0, 0, 0] # heading double dot
                     ]) 

B_matrix = np.array([[0, 0, 0], # x
                     [0, 0, 0], # y
                     [0, 0, 0], # Vx
                     [0, 0, 0], # vy
                     [0, 0, 0], # heading
                     [0, 0, 0], # heading rate
                     [0, 0, 1], # steering angle
                     [1.5, 0, 0], # a
                     [0, 1.35, 0], # heading double dot
                     ]) 

U_matrix = np.zeros((3,1))

C_matrix = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0, 0]])

# I assumed an error of 100 m/s for velocity, 100 m/s^2 for acceleration, and 1 rad for yaw and steering angle

P_matrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0]
                     ])

P_hat_matrix = np.zeros((9,9))

# I assumed a process noise of 0.01 for every state

Q_matrix = np.array([[0.01, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0.01, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0.01, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0.01, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0.01, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0.01, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0.01, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0.01, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0.01]
                     ])

R_matrix = np.array([[0, 0, 0],
                     [0, 0, 0],
                     [0, 0, 0]])


Z_matrix = np.zeros((3,1))

K_matrix = np.zeros_like(np.transpose(C_matrix))

I_matrix = np.identity(9)

### START CODE ###

# std_msgs/Header
# ------------------------- Global Variables ------------------------- #
### Car Parameters ###
radius_wheel = 0.340175 # Radius of wheels
track_width = 2*0.82491 # 1.64982 # (distance between two rear wheels)
wheel_base = 2.26972 # distance between front and rear wheels


position_X_global = 0
position_Y_global = 0

sim_time = 0
new_time = 0
old_time = 0
start_time = 0
elapsed_time = 0
first_time_flag = True

degree_flag = True


# ------------------------- call back functions ------------------------- #

def clock_callback(msg:Clock):
    pass
    global sim_time
    global old_time
    global first_time_flag

    sim_time = msg.clock
    sim_time = sim_time.secs + sim_time.nsecs * 1e-9

    if(first_time_flag):
        old_time = sim_time
        start_time = sim_time
        first_time_flag = False


def odom_callback(initial_states:Odometry):
    pass
    global X_matrix
    global X_hat_matrix
    global P_matrix
    global P_hat_matrix
    global K_matrix
    global B_matrix

    global position_X
    global position_Y

    position_X = initial_states.pose.pose.position.x
    position_Y = initial_states.pose.pose.position.y

    veloxity_X = initial_states.twist.twist.linear.x
    velocity_Y = initial_states.twist.twist.linear.y
    velocity = math.sqrt(veloxity_X**2 + velocity_Y**2)

    yaw_rate = initial_states.twist.twist.angular.z

    steering_angle = 0
    if(yaw_rate == 0):
        steering_angle = 0
    else:
        I_C_R_distance = velocity/yaw_rate
        steering_angle = math.atan(wheel_base/I_C_R_distance)

    yaw = euler_from_quaternion((initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w))[2]
    # if(degree_flag):
    #     yaw_rate = np.rad2deg(yaw_rate) #Change Radians to degrees
    #     yaw = np.rad2deg(yaw) #Change Radians to degrees
    
    X_matrix[0,0] = position_X
    X_matrix[1,0] = position_Y
    X_matrix[2,0] = 0
    X_matrix[3,0] = velocity
    X_matrix[4,0] = yaw
    X_matrix[5,0] = yaw_rate
    X_matrix[6,0] = steering_angle

    odom_subscriber.unregister()

def encoder_callback(msg:Float64MultiArray):
    pass
    ### Global Variables ###   
    global X_matrix
    global Z_matrix
    global position_X_global
    global position_Y_global

    global sim_time
    global new_time
    global old_time

    ## Angular Velocity of the Wheels in rad ###
    FL_omega = abs(msg.data[0])
    FR_omega = abs(msg.data[1])
    RL_omega = abs(msg.data[2]) # R left
    RR_omega = abs(msg.data[3]) # R Right
    
    ## Velocity of wheels: V = ω . R ###
    FL_velocity = FL_omega * radius_wheel
    FR_velocity = FR_omega * radius_wheel
    RL_velocity = RL_omega * radius_wheel
    RR_velocity = RR_omega * radius_wheel

    ### Car Linear & Angular Velocity & Steering Angle###
    velocity = (RL_velocity + RR_velocity)/2
    velocity = 0.995*velocity #TODO REMOVE

    yaw_rate =  (RR_velocity - RL_velocity)/track_width
    yaw_rate = 0.975*yaw_rate #TODO REMOVE

    # Steering Angle
    if(yaw_rate == 0):
        steering_angle = 0
    else:
        I_C_R_distance = velocity/yaw_rate
        steering_angle = math.atan(wheel_base/I_C_R_distance)


    Z_matrix[0,0] =  velocity
    Z_matrix[1,0] =  yaw_rate
    Z_matrix[2,0] =  steering_angle

    U_matrix[0,0] =  velocity
    U_matrix[1,0] =  yaw_rate
    U_matrix[2,0] =  steering_angle

    ### Integrate Velocities to find Postion ###
    ### ------------ Integration START --------------  ###

    new_time = sim_time
    dt = new_time - old_time ##
    old_time = new_time

    # if(dt == 0):
    #     f = 123
    # else:
    #     f = 1/dt


    ### Publish vehicle_velocity Data ###
    ### Linear & Angular Velocities ###
    kalman_filter() 
    vehicle_velocity_publishData = Odometry()

    position_X = X_matrix[0,0]
    position_Y = X_matrix[1,0]

    yaw = X_matrix[4,0]

    # Transform Velocities to Global Frame
    velocity_X = X_matrix[3,0]*math.sin(-yaw)
    velocity_Y = X_matrix[3,0]*math.cos(yaw)

    position_X_global = position_X_global + T * velocity_X
    position_Y_global = position_Y_global + T * velocity_Y

    yaw_rate = X_matrix[5,0]

    steering_angle = X_matrix[6,0]

    if(degree_flag):
        yaw_rate = np.rad2deg(yaw_rate) #Change Radians to degrees
        yaw = np.rad2deg(yaw) #Change Radians to degrees
        


    ## Attach Data
    # Pose
    vehicle_velocity_publishData.pose.pose.position.x = position_X_global
    vehicle_velocity_publishData.pose.pose.position.y = position_Y_global
    vehicle_velocity_publishData.pose.pose.position.z = 0

    vehicle_velocity_publishData.pose.pose.orientation.x = 0
    vehicle_velocity_publishData.pose.pose.orientation.y = 0
    vehicle_velocity_publishData.pose.pose.orientation.z = yaw
    vehicle_velocity_publishData.pose.pose.orientation.w = X_matrix[6,0]

    # Twist
    vehicle_velocity_publishData.twist.twist.linear.x = velocity_X
    vehicle_velocity_publishData.twist.twist.linear.y = velocity_Y
    vehicle_velocity_publishData.twist.twist.linear.z = velocity

    vehicle_velocity_publishData.twist.twist.angular.x = 0
    vehicle_velocity_publishData.twist.twist.angular.y = 0
    vehicle_velocity_publishData.twist.twist.angular.z = yaw_rate

    # ## Publish Data
    encoder_state_estimation_topic_publisher.publish(vehicle_velocity_publishData)

    # rospy.loginfo("Msg")


### END CODE ###
    

def kalman_filter():
    global X_matrix
    global X_hat_matrix
    global P_matrix
    global P_hat_matrix
    global K_matrix
    global B_matrix

    # Predict state ahead
    X_hat_matrix = np.matmul(A_matrix, X_matrix) + np.matmul(B_matrix, U_matrix)

    # Predict Error Covariance
    P_hat_matrix = np.matmul(np.matmul(A_matrix, P_matrix), np.transpose(A_matrix)) + Q_matrix

    # Calculate Kalman Gain
    A = np.matmul(P_hat_matrix, np.transpose(C_matrix))
    B = np.matmul(np.matmul(C_matrix, P_hat_matrix), np.transpose(C_matrix))
    C = np.linalg.inv(B + R_matrix)
    K_matrix = np.matmul(A,C)

    # Update State
    # X_matrix = X_hat_matrix + np.matmul(K_matrix, (Z_matrix - np.matmul(C_matrix, X_hat_matrix))) #TODO Uncomment
    X_matrix = X_hat_matrix

    #Constrain the theta to be within -pi <= theta <= pi
    if(X_matrix[4,0] < -math.pi):
        X_matrix[4,0] += 2*math.pi
    elif( X_matrix[4,0] > math.pi):
        X_matrix[4,0] -+ 2*math.pi

    # Update Error Covariance
    P_matrix = np.matmul((I_matrix - np.matmul(K_matrix, C_matrix)), P_hat_matrix)

    # Update the B_matrix for the next snapshot
    # B_matrix[6,0] = T*np.tan(X_matrix[-1,0])/L
    # B_matrix[7,0] = np.tan(X_matrix[-1,0])/L

    '''
        Before publishing, i gotta convert it from body frame to global frame to compare against ground truth
    '''


    # publish_ground_truth.publish(ground_truth)



if __name__ == '__main__':
    # ------------------------- Global Variables ------------------------- #
    

    # ------------------------- Node initiation ------------------------- #
    rospy.init_node("encoder_se_KF")
    rospy.loginfo("encoder_se_KF Start")


    # ------------------------- Topics ------------------------- #
    wheel_vel_topic = ("/wheel_vel" , Float64MultiArray) # Subscribe
    clock_topic = ('/clock', Clock) # Subscribe
    odom_topic = ('/odom', Odometry) # Subscribe
    # state_estimation_encoder_topic = ('/vehicle_velocities', Vector3) # Publish
    # xyz_estimation_encoder_topic = ('/vehicle_position', Vector3) # Publish
    encoder_state_estimation_topic = ('/encoder_state_estimation', Odometry) # Publish

    # ------------------------- Subscribers ------------------------- #
    
    clock_subscriber = rospy.Subscriber(clock_topic[0], clock_topic[1] , callback=clock_callback)
    rospy.wait_for_message(clock_topic[0], clock_topic[1]) # Manually call clock_callback to initialize sim_time

    odom_subscriber = rospy.Subscriber(odom_topic[0], odom_topic[1] , callback=odom_callback)
    rospy.wait_for_message(odom_topic[0], odom_topic[1]) # Manually call odom_callback to initialize x,y,z

    wheel_vel_subscriber = rospy.Subscriber(wheel_vel_topic[0], wheel_vel_topic[1], callback=encoder_callback)

    # ------------------------- publishers ------------------------- #

    # vehicle_velocity_publisher = rospy.Publisher(state_estimation_encoder_topic[0], state_estimation_encoder_topic[1], queue_size=10)
    # vehicle_position_publisher = rospy.Publisher(xyz_estimation_encoder_topic[0], xyz_estimation_encoder_topic[1], queue_size=10)
    encoder_state_estimation_topic_publisher = rospy.Publisher(encoder_state_estimation_topic[0], encoder_state_estimation_topic[1], queue_size=10)

    # ------------------------- Rest of Code ------------------------- #
    rospy.spin()

    rospy.loginfo("encoder_se_KF Exit")

    rospy.spin()
