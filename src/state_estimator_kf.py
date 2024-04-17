#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_matrix
import time

####################### Global variables declarations #######################

frequency = 60

old_time = 0
new_time = 0

current_quaternion = np.zeros((1,4))

obtained_initial_orientation_flag = False

curr_linear_velocity = Vector3Stamped()

curr_linear_acceleration = Vector3()

past_inputs = np.zeros((3,2))
past_outputs = np.zeros((3,2))

fs =100
T = 1/fs
fc = 2
k = 2/T
# prewarping
w_d = 2*np.pi*fc/fs
wa =k*np.tan(w_d/2)
a = (k*k) - (wa*k*np.sqrt(2)) + (wa*wa)
b = -2*k*k + (2*wa*wa)
c = (k*k) + (wa*k*np.sqrt(2)) + (wa*wa)

A = -b/c
B = -a/c
C = (wa*wa)/c
D = (2*wa*wa)/c
E = (wa*wa)/c

'''
    The Kalman filter consists of 5 equations

    1] Predict state ahead: X_hat = A * K + B * U
    2] Predict Error Covariance: P_hat = A * P * transpose(A) + Q
    3] Calculate Kalman gain: K = P * transpose(C) * inverse(C * P * transpose(C) + R)
    4] Update State: X = X_hat + K * [Z - C * X_hat]
    5] Update Error Covariance: P = [I - K * C] * P_hat

    Now it is important to note that:

    1] X_matrix: The states of the system and they are transpose([vx, vy, vz, ax, ay, az, yaw])
    2] X_hat_matrix: These are the initially estimated states
    3] A_matrix: The state transition matrix
    4] B_matrix: The input mapping matrix
    5] C_matrix: This is the matrix mapping states to outputs
    6] P_matrix: This is the prediction error Covariance matrix
    7] P_hat_matrix: This is the initially estiamted prediction error Covariance matrix
    8] Q_matrix: This is the process noise Covariance matrix. It is assumed to be white gaussian noise
    9] R_matrix: This is the sensor measurement error Covariance matrix. This one was computed via a script analyzing stationary readings
    10] Z_matrix: This is a matrix containing the current sensors' readings
    11] K_matrix: This is the Kalman gains matrix
    12] I_matrix: An Identity Matrix

      Kalman Parameters initialization:
'''
g = 9.81

T = 1/fs

# vx, vy, vz, ax, ay, az, pitch
X_matrix = np.array([[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]])

X_hat_matrix = np.array([[0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]])

A_matrix = np.array([[1, 0, 0, T, 0, 0, 0],
                    [0, 1, 0, 0, T, 0, 0],
                    [0, 0, 1, 0, 0, T, 0],
                    [0, 0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 0, 1]])

B_matrix = np.array([[-g*np.sin(X_matrix[-1,0])*T],
                    [-g*np.sin(X_matrix[-1,0])*T],
                    [-g*np.sin(X_matrix[-1,0])*T],
                    [0],
                    [0],
                    [0],
                    [0]])

C_matrix = np.arrray([[0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 0, 1]])

# I assumed an error of 100 m/s for velocity, 100 m/s^2 for acceleration, and 1 rad for yaw

P_matrix = np.arrray([[100, 0, 0, 0, 0, 0, 0],
                     [0, 100, 0, 0, 0, 0, 0],
                     [0, 0, 100, 0, 0, 0, 0],
                     [0, 0, 0, 100, 0, 0, 0],
                     [0, 0, 0, 0, 100, 0, 0],
                     [0, 0, 0, 0, 0, 100, 0],
                     [0, 0, 0, 0, 0, 0, 1]])

P_hat_matrix = np.arrray([[100, 0, 0, 0, 0, 0, 0],
                     [0, 100, 0, 0, 0, 0, 0],
                     [0, 0, 100, 0, 0, 0, 0],
                     [0, 0, 0, 100, 0, 0, 0],
                     [0, 0, 0, 0, 100, 0, 0],
                     [0, 0, 0, 0, 0, 100, 0],
                     [0, 0, 0, 0, 0, 0, 1]])

# I assumed a process noise of 0.01 for every state

Q_matrix = np.arrray([[0.01, 0, 0, 0, 0, 0, 0],
                     [0, 0.01, 0, 0, 0, 0, 0],
                     [0, 0, 0.01, 0, 0, 0, 0],
                     [0, 0, 0, 0.01, 0, 0, 0],
                     [0, 0, 0, 0, 0.01, 0, 0],
                     [0, 0, 0, 0, 0, 0.01, 0],
                     [0, 0, 0, 0, 0, 0, 0.01]])

R_matrix = np.arrray([[0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 2.427458e-05, -1.669918e-05, 6.829418e-07, 2.204473e-06],
                     [0, 0, 0, -1.669918e-05, 4.997876e-04, 2.045260e-05, -3.472394e-07],
                     [0, 0, 0, 6.829418e-07, 2.045260e-05, 4.234005e-05, 2.749304e-08],
                     [0, 0, 0, 2.204473e-06, -3.472394e-07, 2.749304e-08, 2.407655e-07]])

Z_matrix = np.array([0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0])

K_matrix = np.zeros_like(C_matrix)

I_matrix = np.identity(7)

imu_alignment_matrix = np.zeros((3,3))

def initial_states_callback(initial_states:Odometry):
    global obtained_initial_orientation_flag
    global imu_alignment_matrix
    global X_matrix

    obtained_initial_orientation_flag = True
    
    imu_alignment_matrix = quaternion_matrix((initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w))

    X_matrix[-1,0] = euler_from_quaternion((initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w))[1]

    initial_states_subsrciber.unregister()

def imu_callback(states:Imu):
    global Z_matrix

    global current_quaternion
    global curr_linear_velocity
    global curr_linear_acceleration
    global old_time
    global new_time
    global past_inputs
    global past_outputs

    if obtained_initial_orientation_flag:

        '''
            In order to obtain the lienar velocity, trapezoidal integration is going to be used.
            The formula is new_reading = 0.5 * (current_reading + new_sensor_data) * time_step

            It is important however, to remove the effect of acceleration from the reading. since our
            application is a vehicle, and assuming we are driving on a flat ground, we will be assuming that we are
            driving on a flat ground, therefore, we only add 9.81 from az since acceleration is a vector pointing downwards
        '''

        new_time = time.time()

        curr_linear_acceleration.x = A*past_outputs[(0,1)] + B*past_outputs[(0,0)] + C*states.linear_acceleration.x + D*past_inputs[(0,1)] + E*past_inputs[(0,0)]
        curr_linear_acceleration.y = A*past_outputs[(1,1)] + B*past_outputs[(1,0)] + C*states.linear_acceleration.y + D*past_inputs[(1,1)] + E*past_inputs[(1,0)]
        curr_linear_acceleration.z = A*past_outputs[(2,1)] + B*past_outputs[(2,0)] + C*(states.linear_acceleration.z+9.81) + D*past_inputs[(2,1)] + E*past_inputs[(2,0)]


        # Trapezoidal Integration Butterworth
        curr_linear_velocity.vector.x += 0.5 * (curr_linear_acceleration.x + past_outputs[(0,1)]) * T
        curr_linear_velocity.vector.y += 0.5 * (curr_linear_acceleration.y + past_outputs[(1,1)]) * T
        curr_linear_velocity.vector.z += 0.5 * (curr_linear_acceleration.z + past_outputs[(2,1)]) * T

        # upadte Acceleration butterworth
        past_outputs[(0,0)] = past_outputs[(0,1)]
        past_outputs[(1,0)] = past_outputs[(1,1)]
        past_outputs[(2,0)] = past_outputs[(2,1)]

        past_outputs[(0,1)] = curr_linear_acceleration.x
        past_outputs[(1,1)] = curr_linear_acceleration.y
        past_outputs[(2,1)] = curr_linear_acceleration.z

        past_inputs[(0,0)] = past_inputs[(0,1)]
        past_inputs[(1,0)] = past_inputs[(1,1)]
        past_inputs[(2,0)] = past_inputs[(2,1)]

        past_inputs[(0,1)] = states.linear_acceleration.x
        past_inputs[(1,1)] = states.linear_acceleration.y
        past_inputs[(2,1)] = states.linear_acceleration.z+9.81

        # update time
        old_time = new_time

        rotation = quaternion_matrix([states.orientation.x, states.orientation.y, states.orientation.z, states.orientation.w])[:3,:3]

        velocity = np.array([curr_linear_velocity.vector.x, curr_linear_velocity.vector.y, curr_linear_velocity.vector.z])
        result = np.matmul(rotation, velocity)
        output = Vector3(result[0], result[1], result[2])
        publish_current_linear_velocity.publish(output)

        # publish filtered IMU data
        publish_filtered_imu_data.publish(curr_linear_acceleration)
    

if __name__ == '__main__':

    ####################### Initiating node and obtaining parameters #######################

    rospy.init_node("state_estimator_node")
    rospy.loginfo("state estimator node initialized")

    # frequency = rospy.get_param("~frequency", 'default_value')

    ####################### Topics declarations #######################

    yaw_topic = ("/current_heading", Float64)
    imu_topic = ("/Imu", Imu)
    # linear_velocity_topic = ("/current_linear_velocity", Vector3Stamped)
    linear_velocity_topic = ("/current_linear_velocity", Vector3)

    initial_states_topic = ("/odom", Odometry )

    ####################### Testing Topics #######################

    estimated_orientation_topic = ("/orientation", Quaternion)
    filtered_imu_data_topic = ("/Imu_filtered", Vector3)

    ####################### Node subscriptions #######################

    initial_states_subsrciber = rospy.Subscriber(initial_states_topic[0], initial_states_topic[1], initial_states_callback)
    old_time = time.time()
    imu_subscriber = rospy.Subscriber(imu_topic[0], imu_topic[1], imu_callback)

    ####################### Node publications #######################

    publish_current_heading = rospy.Publisher(yaw_topic[0], yaw_topic[1], queue_size=0)
    publish_current_linear_velocity = rospy.Publisher(linear_velocity_topic[0], linear_velocity_topic[1], queue_size=0)

    ####################### Testing Topics #######################

    publish_estimated_orientation = rospy.Publisher(estimated_orientation_topic[0], estimated_orientation_topic[1], queue_size=0)
    publish_filtered_imu_data = rospy.Publisher(filtered_imu_data_topic[0], filtered_imu_data_topic[1], queue_size=0)
    rospy.spin()