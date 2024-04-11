#!/usr/bin/env python3
import rospy
import math
import time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock



# Global Variables:
PI = math.pi
x = 0
y = 0
z = 0
theta = math.pi/2
old_time = time.time()
new_time = 0
first_time = True
time_zero = time.time()




def encoder_callback(msg:Float64MultiArray):
    pass
    # rospy.loginfo(msg.data)

    r_wheel = 0.340175
    d = 0.82491 # 2*d = track width (distance between two back wheels)
    l = 2.26972 # Wheel Base

    # b = 2*82491
    # l = 2.26972

    # Vel:
    # omega_FL = abs(msg.data[0])
    omega_right = abs(msg.data[3])
    omega_left = abs(msg.data[2])
    
    # v_FL = omega_FL * r_wheel
    v_right = omega_right * r_wheel
    v_left = omega_left * r_wheel
    

    vel = (v_left + v_right)/2

    # Steering:    

    right = False
    v_big = v_right
    v_small = v_left
    if(v_left > v_right):
        v_big = v_left
        v_small = v_right
        right = True
        
    # slope = (v_big - v_small)/2*d
    # R_small = 1/(slope) * v_small
    # steering_angle = math.atan(l / (R_small + d))

    # ratio_omega = 1.78 #TODO Remove
    w_z = (v_big - v_small)/(2*d)
    # w_z = ratio_omega * w_z


    if(right):
        w_z = -1*w_z

    publishData = Vector3()

    publishData.x = vel
    publishData.z = w_z


    vehicle_velocities_publisher.publish(publishData)
    integrate(vel,w_z)

    # rospy.loginfo(vel)

# ---------------------------------------------- #
# Regular Functions
def integrate(vel, omega):
    ratio = 1.6 #TODO Remove
    pass
    global x
    global y
    global theta
    global old_time
    global new_time
    global first_time
    global time_zero

    if(first_time):
        old_time = time.time()
        time_zero = old_time
        first_time = False

    new_time = time.time()
    dt = new_time - old_time
    dt *= ratio
    # dt = 1/60

    vel_x = vel * math.cos(theta) 
    vel_y = vel * math.sin(theta)

    # -------------------  Euler ----------------------- #
    x = x + vel_x*dt
    y = y + vel_y*dt
    theta = theta + omega*dt


    # -------------------  RK4 ----------------------- #
    # k1_x = vel * math.cos(theta) 
    # k1_y = vel * math.sin(theta)
    # 
    # # k1_theta = get_angular_velocity(vel)  

    # k2_x = vel * math.cos(theta + theta * dt / 2)
    # k2_y = vel * math.sin(theta + theta * dt / 2)
    # # k2_theta = get_angular_velocity(vel, k1_x, k1_y)  # Update angular velocity calculation

    # k3_x = vel * math.cos(theta + theta * dt / 2)
    # k3_y = vel * math.sin(theta + theta * dt / 2)
    # # k3_theta = get_angular_velocity(vel, k2_x, k2_y)  # Update angular velocity calculation

    # k4_x = vel * math.cos(theta + theta * dt)
    # k4_y = vel * math.sin(theta + theta * dt)
    # # k4_theta = get_angular_velocity(vel, k3_x, k3_y)  # Update angular velocity calculation

    # x += (k1_x + 2*k2_x + 2*k3_x + k4_x) * dt / 6
    # y += (k1_y + 2*k2_y + 2*k3_y + k4_y) * dt / 6
    # # theta += (k1_theta + 2*k2_theta + 2*k3_theta + k4_theta) * dt / 6

    # --------------------------------------------------
    old_time = new_time
    elapsed = new_time - time_zero

    out = Vector3()
    out.x = x
    out.y = y

    
    if(theta < 0):
        theta += 2*PI
    if(theta >= 2*PI):
        theta -= 2*PI

    theta_out = theta * (180/PI)
    out.z = theta_out

    vehicle_position_publisher.publish(out)


    

if __name__ == "__main__":
    # Node initiation #
    rospy.loginfo("encoder_se Start")
    rospy.init_node("encoder_se")

    # Topics #
    wheel_vel_topic = ("/wheel_vel" , Float64MultiArray)
    state_estimation_encoder_topic = ('/vehicle_velocities', Vector3)
    xyz_estimation_encoder_topic = ('/vehicle_position', Vector3)
    clock = ('/clock', )

    # Subscribers #
    wheel_vel_subscriber = rospy.Subscriber(wheel_vel_topic[0], wheel_vel_topic[1], callback=encoder_callback)

    # publishers #
    vehicle_velocities_publisher = rospy.Publisher(state_estimation_encoder_topic[0], state_estimation_encoder_topic[1], queue_size=10)
    vehicle_position_publisher = rospy.Publisher(xyz_estimation_encoder_topic[0], xyz_estimation_encoder_topic[1], queue_size=10)

    rospy.spin()

    rospy.loginfo("encoder_se Exit")

    duration = new_time - time_zero
    rospy.loginfo(duration)