#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import os
os.environ["OPEN-BLAS_NUM_THREADS"] = "1"
import numpy as np

####################### Global variables declarations #######################

max_velocity = 33.33 # m/s
velocity_change_threashold = 2
fs =100 # Hz
T = 1/fs # s
L = 2.269 # m

'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = f(x,u)
    2] Predict Error Covariance: P_hat = F * P * transpose(F) + Q
    3] Calculate Kalman gain: K = P * transpose(H) * inverse(H * P * transpose(H) + R)
    4] Update State: X = X_hat + K * [Z - h(x)]
    5] Update Error Covariance: P = [I - K * H] * P_hat

    Now it is important to note that:

    1] X_matrix: The states of the system and they are transpose([x, y, z, vx, vy, vz, yaw, steering angle])
    2] X_hat_matrix: These are the initially estimated states
    3] f_matrix (AKA f(x,u)): The state transition matrix
    4] F_matrix: This is the Jacobian of the f_matrix w.r.t the states
    5] h_matrix (AKA h(x)): This is the observations matrix
    6] H_matrix: This is the Jacobian of the h_matrix w.r.t the states
    7] P_matrix: This is the prediction error Covariance matrix
    8] P_hat_matrix: This is the initially estiamted prediction error Covariance matrix
    9] Q_matrix: This is the process noise Covariance matrix. It is assumed to be white gaussian noise
    10] R_matrix: This is the sensor measurement error Covariance matrix. This one was computed via a script analyzing stationary readings
    11] Z_matrix: This is a matrix containing the current sensors' readings
    12] K_matrix: This is the Kalman gains matrix
    13] I_matrix: An Identity Matrix

      Kalman Parameters initialization:
'''

# These are the inputs we need for the f_matrix. Their Values are obtained from the controller outputs callbacks.
Vd = 0
steering_angle = 0


# [x, y, z, vx, vy, vz, yaw, steering angle]T
X_matrix = np.zeros((8,1))

X_hat_matrix = np.zeros((8,1))

f_matrix = np.array([[X_matrix[0,0] - T*Vd*np.sin(X_matrix[-2,0])],
                     [X_matrix[1,0] + T*Vd*np.cos(X_matrix[-2,0])],
                     [X_matrix[2,0]],
                     [-Vd*np.sin(X_matrix[-2,0])],
                     [Vd*np.cos(X_matrix[-2,0])],
                     [0],
                     [X_matrix[-2,0] + T*Vd*np.tan(X_matrix[-1,0])/L],
                     [steering_angle]])

F_matrix = np.array([[1, 0, 0, 0, 0, 0, -T*Vd*np.cos(X_matrix[-2,0]), 0],
                     [0, 1, 0, 0, 0, 0, -T*Vd*np.sin(X_matrix[-2,0]), 0],
                     [0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, -Vd*np.cos(X_matrix[-2,0]), 0],
                     [0, 0, 0, 0, 0, 0, -Vd*np.sin(X_matrix[-2,0]), 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, (T*Vd)/(L*(np.cos(X_matrix[-1,0])**2))],
                     [0, 0, 0, 0, 0, 0, 0, 0]])

h_matrix = np.array([[X_matrix[0,0]],
                     [X_matrix[1,0]],
                     [X_matrix[2,0]],
                     [X_matrix[3,0]],
                     [X_matrix[4,0]],
                     [X_matrix[5,0]],
                     [X_matrix[6,0]]])

H_matrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0]])

# I assumed an error of 100 m/s for velocity, 100 m/s^2 for acceleration, and 1 rad for yaw and steering angle

P_matrix = np.array([[100, 0, 0, 0, 0, 0, 0, 0],
                     [0, 100, 0, 0, 0, 0, 0, 0],
                     [0, 0, 100, 0, 0, 0, 0, 0],
                     [0, 0, 0, 100, 0, 0, 0, 0],
                     [0, 0, 0, 0, 100, 0, 0, 0],
                     [0, 0, 0, 0, 0, 100, 0, 0],
                     [0, 0, 0, 0, 0, 0, 3, 0],
                     [0, 0, 0, 0, 0, 0, 0, 3]])

P_hat_matrix = np.zeros((8,8))

# I assumed a process noise of 0.01 for every state

Q_matrix = np.array([[0.01, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0.01, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0.01, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0.01, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0.01, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0.01, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0.01, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0.01]])

# In case of error was in degrees
# R_matrix = np.array([[(2.269/2),0,0,0,0,0,0],
#                      [0,(1.649/2),0,0,0,0,0],
#                      [0,0,(2.269/2),0,0,0,0],
#                      [0,0,0,(2.269/2),0,0,0],
#                      [0,0,0,0,(1.649/2),0,0],
#                      [0,0,0,0,0,(2.269/2),0],
#                      [0,0,0,0,0,0,np.deg2rad((1.649/2))]])

# In case of error was in rad
R_matrix = np.array([[(2.269/2),0,0,0,0,0,0],
                     [0,(1.649/2),0,0,0,0,0],
                     [0,0,(2.269/2),0,0,0,0],
                     [0,0,0,(2.269/2),0,0,0],
                     [0,0,0,0,(1.649/2),0,0],
                     [0,0,0,0,0,(2.269/2),0],
                     [0,0,0,0,0,0,(1.649/2)]])

Z_matrix = np.zeros((6,1))

K_matrix = np.zeros_like(np.transpose(H_matrix))

I_matrix = np.identity(8)

# Syncronization flags
obtained_vd_flag = False
obtained_steering_flag = False

# syncronization parameter
ground_truth = Odometry()
noisy_data = Odometry()

def odometry_callback(states:Odometry):
    global Z_matrix
    global ground_truth
    global noisy_data
    global obtained_steering_flag
    global obtained_vd_flag

    ground_truth = states

    # Corrupt signal
    ground_truth_yaw = euler_from_quaternion((states.pose.pose.orientation.x, states.pose.pose.orientation.y, states.pose.pose.orientation.z, states.pose.pose.orientation.w))[2]

    # In case of error was in degrees
    # corrupted_ground_truth = np.array([[states.pose.pose.position.x + np.random.normal(0,np.sqrt(2.269/2))],
    #                                   [states.pose.pose.position.y + np.random.normal(0,np.sqrt(1.649/2))],
    #                                   [states.pose.pose.position.z + np.random.normal(0,np.sqrt(2.269/2))],
    #                                   [states.twist.twist.linear.x + np.random.normal(0,np.sqrt(2.269/2))],
    #                                   [states.twist.twist.linear.y + np.random.normal(0,np.sqrt(1.649/2))],
    #                                   [states.twist.twist.linear.z + np.random.normal(0,np.sqrt(2.269/2))],
    #                                   [ground_truth_yaw + np.deg2rad(np.random.normal(0,np.sqrt(1.649/2)))]])
    
    # In case of error was in rad
    corrupted_ground_truth = np.array([[states.pose.pose.position.x + np.random.normal(0,np.sqrt(2.269/2))],
                                      [states.pose.pose.position.y + np.random.normal(0,np.sqrt(1.649/2))],
                                      [states.pose.pose.position.z + np.random.normal(0,np.sqrt(2.269/2))],
                                      [states.twist.twist.linear.x + np.random.normal(0,np.sqrt(2.269/2))],
                                      [states.twist.twist.linear.y + np.random.normal(0,np.sqrt(1.649/2))],
                                      [states.twist.twist.linear.z + np.random.normal(0,np.sqrt(2.269/2))],
                                      [ground_truth_yaw + np.random.normal(0,np.sqrt(1.649/2))]])

    Z_matrix = [[corrupted_ground_truth[0][0]],
                [corrupted_ground_truth[1][0]],
                [corrupted_ground_truth[2][0]],
                [corrupted_ground_truth[3][0]],
                [corrupted_ground_truth[4][0]],
                [corrupted_ground_truth[5][0]],
                [corrupted_ground_truth[6][0]]]
    
    noisy_data.pose.pose.position.x = corrupted_ground_truth[0][0]
    noisy_data.pose.pose.position.y = corrupted_ground_truth[1][0]
    noisy_data.pose.pose.position.z = corrupted_ground_truth[2][0]
    noisy_data.twist.twist.linear.x = corrupted_ground_truth[3][0]
    noisy_data.twist.twist.linear.y = corrupted_ground_truth[4][0]
    noisy_data.twist.twist.linear.z = corrupted_ground_truth[5][0]
    yaw_noisy = quaternion_from_euler(0,0,corrupted_ground_truth[6][0])
    noisy_data.pose.pose.orientation.x = yaw_noisy[0]
    noisy_data.pose.pose.orientation.y = yaw_noisy[1]
    noisy_data.pose.pose.orientation.z = yaw_noisy[2]
    noisy_data.pose.pose.orientation.w = yaw_noisy[3]
    
    if obtained_vd_flag and obtained_steering_flag:
            obtained_steering_flag = False
            obtained_vd_flag = False
            kalman_filter() 

def input_velocity_callback(input_throttle:Float64):
    global Vd
    global obtained_vd_flag

    Vd = input_throttle.data * max_velocity
    obtained_vd_flag = True


def input_steering_angle_callback(input_steering_angle:Float64):
    global steering_angle
    global obtained_steering_flag

    steering_angle = np.deg2rad(input_steering_angle.data)
    obtained_steering_flag = True

def kalman_filter():
    global X_matrix
    global X_hat_matrix
    global P_matrix
    global P_hat_matrix
    global K_matrix
    global f_matrix
    global F_matrix
    global h_matrix

    # Store old velocities
    old_vx = X_matrix[3,0]
    old_vy = X_matrix[4,0]
    old_vz = X_matrix[5,0]

    # Predict state ahead
    f_matrix = np.array([[X_matrix[0,0] - T*Vd*np.sin(X_matrix[-2,0])],
                     [X_matrix[1,0] + T*Vd*np.cos(X_matrix[-2,0])],
                     [X_matrix[2,0]],
                     [-Vd*np.sin(X_matrix[-2,0])],
                     [Vd*np.cos(X_matrix[-2,0])],
                     [0],
                     [X_matrix[-2,0] + T*Vd*np.tan(X_matrix[-1,0])/L],
                     [steering_angle]])
    
    X_hat_matrix = f_matrix

    # Predict Error Covariance
    F_matrix = np.array([[1, 0, 0, 0, 0, 0, -T*Vd*np.cos(X_matrix[-2,0]), 0],
                     [0, 1, 0, 0, 0, 0, -T*Vd*np.sin(X_matrix[-2,0]), 0],
                     [0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, -Vd*np.cos(X_matrix[-2,0]), 0],
                     [0, 0, 0, 0, 0, 0, -Vd*np.sin(X_matrix[-2,0]), 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, (T*Vd)/(L*(np.cos(X_matrix[-1,0])**2))],
                     [0, 0, 0, 0, 0, 0, 0, 0]])
    
    P_hat_matrix = np.matmul(np.matmul(F_matrix, P_matrix), np.transpose(F_matrix)) + Q_matrix

    # Calculate Kalman Gain
    A = np.matmul(P_hat_matrix, np.transpose(H_matrix))
    B = np.matmul(np.matmul(H_matrix, P_hat_matrix), np.transpose(H_matrix))
    C = np.linalg.inv(B + R_matrix)
    K_matrix = np.matmul(A,C)

    # Update State
    h_matrix = np.array([[X_matrix[0,0]],
                     [X_matrix[1,0]],
                     [X_matrix[2,0]],
                     [X_matrix[3,0]],
                     [X_matrix[4,0]],
                     [X_matrix[5,0]],
                     [X_matrix[6,0]]])
    
    X_matrix = X_hat_matrix + np.matmul(K_matrix, (Z_matrix - h_matrix))

    # Check if the velocity had a drastic change
    new_vx = X_matrix[3,0]
    new_vy = X_matrix[4,0]
    new_vz = X_matrix[5,0]
    X_matrix[3,0] = X_matrix[3,0] if abs(new_vx-old_vx)<velocity_change_threashold else old_vx
    X_matrix[4,0] = X_matrix[4,0] if abs(new_vy-old_vy)<velocity_change_threashold else old_vy
    X_matrix[5,0] = X_matrix[5,0] if abs(new_vz-old_vz)<velocity_change_threashold else old_vz

    # Update Error Covariance
    P_matrix = np.matmul((I_matrix - np.matmul(K_matrix, H_matrix)), P_hat_matrix)
    
    '''
        publishing
    '''
    position = np.array([X_matrix[0,0], X_matrix[1,0], X_matrix[2,0]])
    publish_current_position.publish(Vector3(position[0], position[1], position[2]))

    velocity = np.array([X_matrix[3,0], X_matrix[4,0], X_matrix[5,0]])
    publish_current_linear_velocity.publish(Vector3(velocity[0], velocity[1], velocity[2]))

    heading = X_matrix[-2,0]
    publish_current_heading.publish(Float64(heading))

    publish_ground_truth.publish(ground_truth)
    publish_noisy_data.publish(noisy_data)


if __name__ == '__main__':

    ####################### Initiating node and obtaining parameters #######################

    rospy.init_node("state_estimator_node")
    rospy.loginfo("state estimator node initialized")

    ####################### Topics declarations #######################

    odometry_topic = ("/odom", Odometry)

    input_velocity_topic = ("/cmd_vel", Float64)
    input_steering_angle_topic = ("/SteeringAngle", Float64)

    position_topic = ("/current_position", Vector3)
    linear_velocity_topic = ("/current_linear_velocity", Vector3)
    heading_topic = ("/current_heading", Float64)

    ####################### Syncronization topic #######################

    ground_truth_topic = ("/ground_truth", Odometry)
    noisy_data_topic = ("/noisy_readings", Odometry)

    ####################### Node subscriptions #######################

    odometry_subsrciber = rospy.Subscriber(odometry_topic[0], odometry_topic[1], odometry_callback)

    input_velocity_subscriber = rospy.Subscriber(input_velocity_topic[0], input_velocity_topic[1], input_velocity_callback)
    input_steering_angle_subscriber = rospy.Subscriber(input_steering_angle_topic[0], input_steering_angle_topic[1], input_steering_angle_callback)

    ####################### Node publications #######################

    publish_current_heading = rospy.Publisher(heading_topic[0], heading_topic[1], queue_size=0)
    publish_current_position = rospy.Publisher(position_topic[0], position_topic[1], queue_size=0)
    publish_current_linear_velocity = rospy.Publisher(linear_velocity_topic[0], linear_velocity_topic[1], queue_size=0)

    ####################### Node syncronization #######################
    publish_ground_truth = rospy.Publisher(ground_truth_topic[0],ground_truth_topic[1],queue_size=0)
    publish_noisy_data = rospy.Publisher(noisy_data_topic[0], noisy_data_topic[1], queue_size=0)

    rospy.spin()