#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_matrix, euler_matrix

####################### Global variables declarations #######################

obtained_initial_orientation_flag = False
max_velocity = 33.33 # m/s
fs =100 # Hz
g = 9.81 # m/s^2
T = 1/fs # s
L = 2.269 # m

'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = A * X + B * U
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

# [vx, vy, vz, ax, ay, az, yaw, Yaw_dot, steering angle]T
X_matrix = np.zeros((9,1))

X_hat_matrix = np.zeros((9,1))

A_matrix = np.array([[0, 0, 0, T, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, T, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, T, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 0]])

B_matrix = np.array([[0, 0],
                     [1, 0],
                     [0, 0],
                     [0, 0],
                     [0, 0],
                     [0, 0],
                     [T*np.tan(X_matrix[-1,0])/L, 0],
                     [np.tan(X_matrix[-1,0])/L, 0],
                     [0, 1]])

U_matrix = np.zeros((2,1))

C_matrix = np.array([[0, 0, 0, 1, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 1, 0]])

# I assumed an error of 100 m/s for velocity, 100 m/s^2 for acceleration, and 1 rad for yaw and steering angle

P_matrix = np.array([[100, 0, 0, 0, 0, 0, 0, 0, 0],
                     [0, 100, 0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 100, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 100, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 100, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 100, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 3, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0, 10, 0],
                     [0, 0, 0, 0, 0, 0, 0, 0, 3]])

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
                     [0, 0, 0, 0, 0, 0, 0, 0, 0.01]])

R_matrix = np.array([[2.427458e-05, -1.669918e-05, 6.829418e-07, 6.240109e-08, -2.451355e-09],
                     [-1.669918e-05, 4.997876e-04, 2.045260e-05, -1.338288e-08, 8.412110e-11],
                     [6.829418e-07, 2.045260e-05, 4.234005e-05, 1.572766e-09, -3.898079e-11],
                     [6.240109e-08, -1.338288e-08, 1.572766e-09, 2.385820e-10, -3.818944e-12],
                     [-2.451355e-09, 8.412110e-11, -3.898079e-11, -3.818944e-12, 5.429372e-12]])


Z_matrix = np.zeros((5,1))

K_matrix = np.zeros_like(np.transpose(C_matrix))

I_matrix = np.identity(9)

imu_alignment_matrix = np.zeros((3,3))

# Syncronization flags
obtained_vd_flag = False
obtained_steering_flag = False

# Validation parameter
body_to_global_rotation_matrix = np.zeros((3,3))

# syncronization parameter
ground_truth = Odometry()
first_entry = True

def initial_states_callback(initial_states:Odometry):
    global obtained_initial_orientation_flag
    global imu_alignment_matrix
    global X_matrix
    global first_entry
    global ground_truth

    if first_entry:
        first_entry = False
        obtained_initial_orientation_flag = True

        imu_alignment_matrix = quaternion_matrix((initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w))[:3,:3]

        X_matrix[-3,0] = euler_from_quaternion((initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w))[2]

    #initial_states_subsrciber.unregister()
    ground_truth = initial_states

def imu_callback(imu_readings:Imu):
    global Z_matrix
    global body_to_global_rotation_matrix
    global obtained_steering_flag
    global obtained_vd_flag

    if obtained_initial_orientation_flag:

        '''
            In this step, we are collecting the new sensor readings and storing them in the Z_matrix for the kalman filter
            operations. It is important however, to remove the effect of acceleration from the reading. since our application
            is a vehicle, and assuming we are driving on a flat ground, we only add 9.81 from az since acceleration is a vector
            pointing downwards
        '''

        '''
            Right now i am ignoring the imu alignment since it is almost perfectly alligned
        '''
        # Perform the alignment step using a = R*a_raw and orientation = R*orientation_raw

        imu_linear_accelerations = np.array([imu_readings.linear_acceleration.x, imu_readings.linear_acceleration.y, imu_readings.linear_acceleration.z])
        aligned_imu_linear_accelerations = np.matmul(imu_alignment_matrix, imu_linear_accelerations)

        imu_angular_velocities = np.array([imu_readings.angular_velocity.x, imu_readings.angular_velocity.x, imu_readings.angular_velocity.z])
        aligned_imu_angular_velocities = np.matmul(imu_alignment_matrix, imu_angular_velocities)

        imu_orientation = euler_from_quaternion((imu_readings.orientation.x, imu_readings.orientation.y, imu_readings.orientation.z, imu_readings.orientation.w))
        aligned_imu_orientation = np.matmul(imu_alignment_matrix, imu_orientation)

        Z_matrix = np.array([[aligned_imu_linear_accelerations[0] - 2*X_matrix[1,0]*X_matrix[-2,0]],
                             [aligned_imu_linear_accelerations[1] + 2*X_matrix[0,0]*X_matrix[-2,0]],
                             [aligned_imu_linear_accelerations[2] + g],
                             [aligned_imu_orientation[2]],
                             [aligned_imu_angular_velocities[2]]])

        body_to_global_rotation_matrix = euler_matrix(aligned_imu_orientation[0], aligned_imu_orientation[1], aligned_imu_orientation[2])[:3,:3]

        # Comment the above section and uncomment the section below incase it is desired to remove imu alignment

        # imu_yaw = euler_from_quaternion((imu_readings.orientation.x, imu_readings.orientation.y, imu_readings.orientation.z, imu_readings.orientation.w))[2]
        
        # Obtain the raw readings
        # Z_matrix = np.array([[imu_readings.linear_acceleration.x - 2*X_matrix[1,0]*X_matrix[-2,0]],
        #                      [imu_readings.linear_acceleration.y + 2*X_matrix[0,0]*X_matrix[-2,0]],
        #                      [imu_readings.linear_acceleration.z + g],
        #                      [imu_yaw],
        #                      [imu_readings.angular_velocity.z]])
        
        # body_to_global_rotation_matrix = quaternion_matrix([imu_readings.orientation.x, imu_readings.orientation.y, imu_readings.orientation.z, imu_readings.orientation.w])[:3,:3]

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
    B_matrix[6,0] = T*np.tan(X_matrix[-1,0])/L
    B_matrix[7,0] = np.tan(X_matrix[-1,0])/L

    '''
        Before publishing, i gotta convert it from body frame to global frame to compare against ground truth
    '''
    velocity = np.array([X_matrix[0,0], X_matrix[1,0], X_matrix[2,0]])
    rotated_velocity = np.matmul(body_to_global_rotation_matrix, velocity)
    publish_current_linear_velocity.publish(Vector3(rotated_velocity[0], rotated_velocity[1], rotated_velocity[2]))

    heading = np.array([0, 0, X_matrix[-3,0]])
    rotated_heading = np.matmul(body_to_global_rotation_matrix, heading)
    publish_current_heading.publish(Float64(rotated_heading[2]))

    publish_current_angular_velocity.publish(X_matrix[-2,0])

    publish_ground_truth.publish(ground_truth)



if __name__ == '__main__':

    ####################### Initiating node and obtaining parameters #######################

    rospy.init_node("state_estimator_node")
    rospy.loginfo("state estimator node initialized")

    ####################### Topics declarations #######################

    initial_states_topic = ("/odom", Odometry)
    imu_topic = ("/Imu", Imu)

    input_velocity_topic = ("/cmd_vel", Float64)
    input_steering_angle_topic = ("/SteeringAngle", Float64)

    linear_velocity_topic = ("/current_linear_velocity", Vector3)
    heading_topic = ("/current_heading", Float64)
    angular_velocity_topic = ("/current_angular_velocity", Float64)

    ####################### Syncronization topic #######################

    ground_truth_topic = ("/ground_truth", Odometry)

    ####################### Node subscriptions #######################

    initial_states_subsrciber = rospy.Subscriber(initial_states_topic[0], initial_states_topic[1], initial_states_callback)
    imu_subscriber = rospy.Subscriber(imu_topic[0], imu_topic[1], imu_callback)

    input_velocity_subscriber = rospy.Subscriber(input_velocity_topic[0], input_velocity_topic[1], input_velocity_callback)
    input_steering_angle_subscriber = rospy.Subscriber(input_steering_angle_topic[0], input_steering_angle_topic[1], input_steering_angle_callback)

    ####################### Node publications #######################

    publish_current_heading = rospy.Publisher(heading_topic[0], heading_topic[1], queue_size=0)
    publish_current_linear_velocity = rospy.Publisher(linear_velocity_topic[0], linear_velocity_topic[1], queue_size=0)
    publish_current_angular_velocity = rospy.Publisher(angular_velocity_topic[0], angular_velocity_topic[1], queue_size=0)

    ####################### Node syncronization #######################
    publish_ground_truth = rospy.Publisher(ground_truth_topic[0],ground_truth_topic[1],queue_size=0)

    rospy.spin()
