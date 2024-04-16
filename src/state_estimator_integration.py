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


def initial_states_callback(initial_states:Odometry):
    global current_quaternion
    global obtained_initial_orientation_flag

    obtained_initial_orientation_flag = True
    
    current_quaternion = np.array([initial_states.pose.pose.orientation.x, initial_states.pose.pose.orientation.y, initial_states.pose.pose.orientation.z, initial_states.pose.pose.orientation.w])
    initial_states_subsrciber.unregister()

def imu_callback(states:Imu):
    global current_quaternion
    global curr_linear_velocity
    global curr_linear_acceleration
    global old_time
    global new_time

    if obtained_initial_orientation_flag:

        '''
            This is following the rule that q_new = q_0 + t/2 * w * q_0
            where w is the angular velocity in quaternion representation
            w = [wx,wy,wz,0], following the quat orientation of [x,y,z,w]
        '''
        c = 0.5 / frequency
        omega_in_quat = np.array([states.angular_velocity.x * c, states.angular_velocity.y * c, states.angular_velocity.z * c, 0])

        delta_quaternion = quaternion_multiply(omega_in_quat, current_quaternion)

        current_quaternion = current_quaternion + delta_quaternion
        current_quaternion = current_quaternion / np.linalg.norm(current_quaternion)
        
        current_heading = euler_from_quaternion(current_quaternion)[2]
        publish_current_heading.publish(current_heading)

        # Tester publication
        estimated_quaternion = Quaternion(current_quaternion[0], current_quaternion[1], current_quaternion[2], current_quaternion[3])
        publish_estimated_orientation.publish(estimated_quaternion)

        '''
            In order to obtain the lienar velocity, trapezoidal integration is going to be used.
            The formula is new_reading = 0.5 * (current_reading + new_sensor_data) * time_step

            It is important however, to remove the effect of acceleration from the reading. since our
            application is a vehicle, and assuming we are driving on a flat ground, we will be assuming that we are
            driving on a flat ground, therefore, we only add 9.81 from az since acceleration is a vector pointing downwards
        '''

        # curr_linear_velocity.header.stamp = rospy.Time.now()

        new_time = time.time()

        # Exponential moving average low pass filter (EMA)
        # alpha = 0.5
        # new_acceleration_np = np.array([states.linear_acceleration.x, states.linear_acceleration.y, states.linear_acceleration.z+9.81])
        # curr_acceleration_np = np.array([curr_linear_acceleration.x, curr_linear_acceleration.y, curr_linear_acceleration.z])
        # new_flitered_linear_acceleration = alpha * new_acceleration_np + (1-alpha) * curr_acceleration_np

        # # Trapezoidal Integration
        curr_linear_velocity.vector.y += 0.5 * (curr_linear_acceleration.y + states.linear_acceleration.y) *(new_time - old_time)
        curr_linear_velocity.vector.x += 0.5 * (curr_linear_acceleration.x + states.linear_acceleration.x) *(new_time - old_time)
        curr_linear_velocity.vector.z += 0.5 * (curr_linear_acceleration.z + states.linear_acceleration.z + 9.81) *(new_time - old_time)

        # Trapezoidal Integration EMA
        # curr_linear_velocity.vector.y += 0.5 * (curr_linear_acceleration.y + new_flitered_linear_acceleration[1]) *(new_time - old_time)
        # curr_linear_velocity.vector.x += 0.5 * (curr_linear_acceleration.x + new_flitered_linear_acceleration[0]) *(new_time - old_time)
        # curr_linear_velocity.vector.z += 0.5 * (curr_linear_acceleration.z + new_flitered_linear_acceleration[2]) *(new_time - old_time)

        # Euler Approximation
        # curr_linear_velocity.vector.y += states.linear_acceleration.y * (new_time - old_time)
        # curr_linear_velocity.vector.x += states.linear_acceleration.x * (new_time - old_time)
        # curr_linear_velocity.vector.z += (states.linear_acceleration.z + 9.81) * (new_time - old_time)

        # RK4
        # rk4_integration(states.linear_acceleration, (new_time-old_time))

        # Update Acceleration
        curr_linear_acceleration.y = states.linear_acceleration.y
        curr_linear_acceleration.x = states.linear_acceleration.x
        curr_linear_acceleration.z = states.linear_acceleration.z + 9.81

        # Update Acceleration EMA
        # curr_linear_acceleration.y = new_flitered_linear_acceleration[1]
        # curr_linear_acceleration.x = new_flitered_linear_acceleration[0]
        # curr_linear_acceleration.z = new_flitered_linear_acceleration[2]

        # update time
        old_time = new_time

        rotation = quaternion_matrix([states.orientation.x, states.orientation.y, states.orientation.z, states.orientation.w])[:3,:3]

        velocity = np.array([curr_linear_velocity.vector.x, curr_linear_velocity.vector.y, curr_linear_velocity.vector.z])
        result = np.matmul(rotation, velocity)
        output = Vector3(result[0], result[1], result[2])
        publish_current_linear_velocity.publish(output)

        # publish filtered IMU data
        publish_filtered_imu_data.publish(curr_linear_acceleration)

'''
    This is a function to perform numerical integration using the fourth order Runge-kutta method

    y = v_dot = a, where a is the new reading
'''
def rk4_integration(new_reading:Vector3, TS):
    global curr_linear_velocity

    new_reading_np = np.array([new_reading.x, new_reading.y, new_reading.z + 9.81])
    curr_velocity_np = np.array([curr_linear_velocity.vector.x, curr_linear_velocity.vector.y, curr_linear_velocity.vector.z])

    # getting K1
    k1 = new_reading_np

    # getting k2
    x11 = new_reading_np + (TS/2)*k1
    k2 = x11

    # getting K3
    x12 = new_reading_np + (TS/2)*k2
    k3 = x12

    # getting k4
    x13 = new_reading_np + TS*k3
    k4 = x13

    x_next = curr_velocity_np + (TS/6)*(k1 + 2*k2 + 2*k3 + k4)

    curr_linear_velocity.vector.x = x_next[0]
    curr_linear_velocity.vector.y = x_next[1]
    curr_linear_velocity.vector.z = x_next[2]
    



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
    # publisher_rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     publisher_rate.sleep()
    rospy.spin()