#!/usr/bin/env python3

# Limit the number of threads NumPy's underlying LA library has access to
import os
os.environ["OPENBLAS_NUM_THREADS"] = "1"

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

####################### Global variables declarations #######################

wheelbase = rospy.get_param('~wheelbase', 2.269)
sample_time = -1
odom_time_prev = None
velocity_change_threshold = 2


'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = A * X + B * U
    2] Predict Error Covariance: P_hat = A * P * transpose(A) + Q
    3] Calculate Kalman gain: K = P * transpose(C) * inverse(C * P * transpose(C) + R)
    4] Update State: X = X_hat + K * [Z - C * X_hat]
    5] Update Error Covariance: P = [I - K * C] * P_hat

    Now it is important to note that:

    1] X_matrix: The states of the system and they are transpose([x, y, v, yaw, steering angle])
    2] X_hat_matrix: These are the initially estimated states
    3] A_matrix: The state transition matrix
    4] B_matrix: The input mapping matrix
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

# [x, y, yaw, steering angle]T
X_matrix = np.zeros((4,1))
# X_matrix = np.array([[0.0], [-73.8655], [0.0], [0.0]])

X_hat_matrix = np.zeros((4,1))
# X_hat_matrix = np.array([[0.0], [-73.8655], [0.0], [0.0]])

A_matrix = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 0]])

B_matrix = np.array([[-sample_time*np.sin(X_matrix[-2,0]), 0],
                     [sample_time*np.cos(X_matrix[-2,0]), 0],
                     [sample_time*np.tan(X_matrix[-1,0])/wheelbase, 0],
                     [0, 1]])

U_matrix = np.zeros((2,1))

C_matrix = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0]])

P_matrix = np.zeros((4,4))

P_hat_matrix = np.zeros((4,4))

Q_matrix = np.array([[0.01, 0, 0, 0],
                     [0, 0.01, 0, 0],
                     [0, 0, 0.001, 0],
                     [0, 0, 0, 0.01]])

# In case of error was in degrees
R_matrix = np.array([[(2.269/2), 0, 0],
                     [0, (1.649/2), 0],
                     [0, 0, np.deg2rad(1.649/2)]])

Z_matrix = np.zeros((3,1))

K_matrix = np.zeros_like(np.transpose(C_matrix))

I_matrix = np.identity(4)

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
    global sample_time
    global odom_time_prev

    # Update sampling time
    curr_time = states.header.stamp
    if odom_time_prev is not None:
        sample_time = (curr_time - odom_time_prev).to_sec()
    odom_time_prev = curr_time

    ground_truth = states

    # Corrupt signal
    ground_truth_yaw = euler_from_quaternion((states.pose.pose.orientation.x, states.pose.pose.orientation.y, states.pose.pose.orientation.z, states.pose.pose.orientation.w))[2]

    # In case of error was in degrees
    corrupted_ground_truth = np.array([[states.pose.pose.position.x + np.random.normal(0,np.sqrt(2.269/2))],
                                      [states.pose.pose.position.y + np.random.normal(0,np.sqrt(1.649/2))],
                                      [ground_truth_yaw + np.random.normal(0,np.sqrt(np.deg2rad(1.649/2)))]])    

    Z_matrix = [[corrupted_ground_truth[0][0]],
                [corrupted_ground_truth[1][0]],
                [corrupted_ground_truth[2][0]]]
    
    noisy_data.header.stamp = ground_truth.header.stamp
    noisy_data.pose.pose.position.x = corrupted_ground_truth[0][0]
    noisy_data.pose.pose.position.y = corrupted_ground_truth[1][0]
    yaw_noisy = quaternion_from_euler(0,0,corrupted_ground_truth[2][0])
    noisy_data.pose.pose.orientation.x = yaw_noisy[0]
    noisy_data.pose.pose.orientation.y = yaw_noisy[1]
    noisy_data.pose.pose.orientation.z = yaw_noisy[2]
    noisy_data.pose.pose.orientation.w = yaw_noisy[3]
    
    if (sample_time > 0) and obtained_vd_flag and obtained_steering_flag:
        obtained_steering_flag = False
        obtained_vd_flag = False
        kalman_filter()
    
    # Publish odometry messages
    publish_msgs()

def input_velocity_callback(input_throttle:Float64):
    global U_matrix
    global obtained_vd_flag

    U_matrix[0,0] = input_throttle.data
    obtained_vd_flag = True


def input_steering_angle_callback(input_steering_angle:Float64):
    global U_matrix
    global obtained_steering_flag

    U_matrix[1,0] = np.deg2rad(input_steering_angle.data)
    obtained_steering_flag = True

def kalman_filter():
    global X_matrix
    global X_hat_matrix
    global P_matrix
    global P_hat_matrix
    global K_matrix
    global B_matrix

    # Predict state ahead
    X_hat_matrix = np.matmul(A_matrix, X_matrix) + np.matmul(B_matrix, U_matrix)
    X_hat_matrix[2,0] = normalize_angle(X_hat_matrix[2,0])

    # Predict Error Covariance
    P_hat_matrix = np.matmul(np.matmul(A_matrix, P_matrix), np.transpose(A_matrix)) + Q_matrix

    # Calculate Kalman Gain
    A = np.matmul(P_hat_matrix, np.transpose(C_matrix))
    B = np.matmul(np.matmul(C_matrix, P_hat_matrix), np.transpose(C_matrix))
    C = np.linalg.inv(B + R_matrix)
    K_matrix = np.matmul(A,C)

    # Update State
    Z_tilde_matrix = Z_matrix - np.matmul(C_matrix, X_hat_matrix)
    Z_tilde_matrix[2,0] = normalize_angle(Z_tilde_matrix[2,0])
    X_matrix = X_hat_matrix + np.matmul(K_matrix, Z_tilde_matrix)
    X_matrix[2,0] = normalize_angle(X_matrix[2,0])

    # Update Error Covariance
    P_matrix = np.matmul((I_matrix - np.matmul(K_matrix, C_matrix)), P_hat_matrix)

    # Update the B_matrix for the next snapshot
    B_matrix[0,0] = -sample_time*np.sin(X_matrix[2,0])
    B_matrix[1,0] = sample_time*np.cos(X_matrix[2,0])
    B_matrix[2,0] = sample_time*np.tan(X_matrix[3,0])/wheelbase

def publish_msgs():
    '''
        publishing
    '''
    odometry_estimate_msg = Odometry()
    odometry_estimate_msg.header.stamp = ground_truth.header.stamp

    # Set position field
    position = np.array([X_matrix[0,0], X_matrix[1,0]])
    odometry_estimate_msg.pose.pose.position.x = position[0]
    odometry_estimate_msg.pose.pose.position.y = position[1]
    odometry_estimate_msg.pose.pose.position.z = ground_truth.pose.pose.position.z

    # Set orientation field by converting heading into a quaternion
    heading = X_matrix[2,0]
    quat = quaternion_from_euler(0.0, 0.0, heading)
    odometry_estimate_msg.pose.pose.orientation.x = quat[0]
    odometry_estimate_msg.pose.pose.orientation.y = quat[1]
    odometry_estimate_msg.pose.pose.orientation.z = quat[2]
    odometry_estimate_msg.pose.pose.orientation.w = quat[3]

    # Set velocity field
    odometry_estimate_msg.twist.twist.linear.x = ground_truth.twist.twist.linear.x
    odometry_estimate_msg.twist.twist.linear.y = ground_truth.twist.twist.linear.y
    
    publish_odometry_estimate.publish(odometry_estimate_msg)
    publish_ground_truth.publish(ground_truth)
    publish_noisy_data.publish(noisy_data)

# Function for normalizing angles in rad to range from [-π, π]
def normalize_angle(angle):
    angle = np.fmod(angle, 2*np.pi)  # Normalize angle to be within [0, 2π]
    
    if angle > np.pi:  # Shift to [-π, π] if necessary
        angle -= 2.0 * np.pi
    elif angle<-np.pi:
        angle+= 2*np.pi    
    return angle

if __name__ == '__main__':

    ####################### Initiating node and obtaining parameters #######################

    rospy.init_node("state_estimator_node")
    rospy.loginfo("state estimator node initialized")

    ####################### Topics declarations #######################

    odometry_topic_name = rospy.get_param("~odometry_topic_name", "/odometry")
    input_velocity_topic_name = rospy.get_param("~velocity_topic_name", "/velocity")
    input_steering_topic_name = rospy.get_param("~steering_topic_name", "/steering")

    odometry_estimate_topic = ("/state_estimator/odometry", Odometry)

    ####################### Syncronization topic #######################

    ground_truth_topic = ("/ground_truth", Odometry)
    noisy_data_topic = ("/noisy_readings", Odometry)

    ####################### Node subscriptions #######################

    odometry_subsrciber = rospy.Subscriber(odometry_topic_name, Odometry, odometry_callback)
    input_velocity_subscriber = rospy.Subscriber(input_velocity_topic_name, Float64, input_velocity_callback)
    input_steering_angle_subscriber = rospy.Subscriber(input_steering_topic_name, Float64, input_steering_angle_callback)

    ####################### Node publications #######################

    publish_odometry_estimate = rospy.Publisher(odometry_estimate_topic[0], odometry_estimate_topic[1], queue_size=0)

    ####################### Node syncronization #######################
    publish_ground_truth = rospy.Publisher(ground_truth_topic[0],ground_truth_topic[1],queue_size=0)
    publish_noisy_data = rospy.Publisher(noisy_data_topic[0], noisy_data_topic[1], queue_size=0)

    rospy.spin()
