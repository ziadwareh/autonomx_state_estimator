#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

####################### Global variables declarations #######################

max_velocity = 33.33 # m/s
fs =100 # Hz
T = 1/fs # s
L = 2.269 # m

# Butterworth filter parameters
fc = 1
k = 1/T
# prewarping
w_d = 2*np.pi*fc/fs
wa =k*np.tan(w_d/2)
a = (k*k) - (wa*k*np.sqrt(2)) + (wa*wa)
b = -2*(k*k) + (2*wa*wa)
c = (k*k) + (wa*k*np.sqrt(2)) + (wa*wa)

A = -b/c
B = -a/c
C = (wa*wa)/c
D = (2*wa*wa)/c
E = (wa*wa)/c

heading_outputs = np.zeros(2)
heading_inputs = np.zeros(2)

'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = A * K + B * U
    2] Predict Error Covariance: P_hat = A * P * transpose(A) + Q
    3] Calculate Kalman gain: K = P * transpose(C) * inverse(C * P * transpose(C) + R)
    4] Update State: X = X_hat + K * [Z - C * X_hat]
    5] Update Error Covariance: P = [I - K * C] * P_hat

    Now it is important to note that:

    1] X_matrix: The states of the system and they are transpose([vx, vy, vz, ax, ay, az, yaw, yaw_dot, steering angle])
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

# [x, y, z, vx, vy, vz, yaw, steering angle]T
X_matrix = np.zeros((8,1))

X_hat_matrix = np.zeros((8,1))

A_matrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0]])

B_matrix = np.array([[-T*np.sin(X_matrix[-2,0]), 0],
                     [T*np.cos(X_matrix[-2,0]), 0],
                     [0, 0],
                     [-np.sin(X_matrix[-2,0]), 0],
                     [np.cos(X_matrix[-2,0]), 0],
                     [0, 0],
                     [T*np.tan(X_matrix[-1,0])/L, 0],
                     [0, 1]])

U_matrix = np.zeros((2,1))

C_matrix = np.array([[1, 0, 0, 0, 0, 0, 0, 0],
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

R_matrix = np.array([[2.269/2,0,0,0,0,0,0],
                     [0,1.649/2,0,0,0,0,0],
                     [0,0,2.269/2,0,0,0,0],
                     [0,0,0,(2.269/2)*1,0,0,0],
                     [0,0,0,0,1.649/2,0,0],
                     [0,0,0,0,0,(2.269/2)*2,0],
                     [0,0,0,0,0,0,(1.649/2)*2]])


Z_matrix = np.zeros((6,1))

K_matrix = np.zeros_like(np.transpose(C_matrix))

I_matrix = np.identity(8)

# Syncronization flags
obtained_vd_flag = False
obtained_steering_flag = False

# syncronization parameter
ground_truth = Odometry()
noisy_data = Odometry()
filtered_heading = 0

def odometry_callback(states:Odometry):
    global Z_matrix
    global ground_truth
    global noisy_data
    global obtained_steering_flag
    global obtained_vd_flag

    global heading_inputs
    global heading_outputs
    global filtered_heading

    ground_truth = states

    # Corrupt signal
    ground_truth_yaw = euler_from_quaternion((states.pose.pose.orientation.x, states.pose.pose.orientation.y, states.pose.pose.orientation.z, states.pose.pose.orientation.w))[2]

    corrupted_ground_truth = np.array([[states.pose.pose.position.x + np.random.normal(0,2.269/2)],
                                      [states.pose.pose.position.y + np.random.normal(0,1.649/2)],
                                      [states.pose.pose.position.z + np.random.normal(0,2.269/2)],
                                      [states.twist.twist.linear.x + np.random.normal(0,2.269/2)],
                                      [states.twist.twist.linear.y + np.random.normal(0,1.649/2)],
                                      [states.twist.twist.linear.z + np.random.normal(0,2.269/2)],
                                      [ground_truth_yaw + np.random.normal(0,1.649/2)]])
    
    filtered_heading = A*heading_outputs[1] + B*heading_outputs[0] + C*corrupted_ground_truth[6][0] + D*heading_inputs[1] + E*heading_inputs[0]

    heading_outputs[0] = heading_outputs[1]
    heading_outputs[1] = filtered_heading

    heading_inputs[0] = heading_inputs[1]
    heading_inputs[1] = corrupted_ground_truth[6][0]

    Z_matrix = [[corrupted_ground_truth[0][0]],
                [corrupted_ground_truth[1][0]],
                [corrupted_ground_truth[2][0]],
                [corrupted_ground_truth[3][0]],
                [corrupted_ground_truth[4][0]],
                [corrupted_ground_truth[5][0]],
                [filtered_heading]]
    
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
    global U_matrix
    global obtained_vd_flag

    U_matrix[0,0] = input_throttle.data * max_velocity
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

    # Predict Error Covariance
    P_hat_matrix = np.matmul(np.matmul(A_matrix, P_matrix), np.transpose(A_matrix)) + Q_matrix

    # Calculate Kalman Gain
    A = np.matmul(P_hat_matrix, np.transpose(C_matrix))
    B = np.matmul(np.matmul(C_matrix, P_hat_matrix), np.transpose(C_matrix))
    C = np.linalg.inv(B + R_matrix)
    K_matrix = np.matmul(A,C)

    # Update State
    X_matrix = X_hat_matrix + np.matmul(K_matrix, (Z_matrix - np.matmul(C_matrix, X_hat_matrix)))

    # Update Error Covariance
    P_matrix = np.matmul((I_matrix - np.matmul(K_matrix, C_matrix)), P_hat_matrix)

    # Update the B_matrix for the next snapshot
    B_matrix[0,0] = -T*np.sin(X_matrix[-2,0])
    B_matrix[1,0] = T*np.cos(X_matrix[-2,0])
    B_matrix[3,0] = -np.sin(X_matrix[-2,0])
    B_matrix[4,0] = np.cos(X_matrix[-2,0])
    B_matrix[6,0] = T*np.tan(X_matrix[-1,0])/L
    
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
    publish_filtered_heading.publish(Float64(filtered_heading))


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
    filtered_heading_data_topic = ("/filtered_heading", Float64)

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
    publish_filtered_heading = rospy.Publisher(filtered_heading_data_topic[0], filtered_heading_data_topic[1], queue_size=0)

    rospy.spin()