#!/usr/bin/env python3
import rospy
import math
import time

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

from rosgraph_msgs.msg import Clock

from nav_msgs.msg import Odometry


# std_msgs/Header
# ------------------------- Global Variables ------------------------- #
PI = math.pi
x = 0
y = 0
z = 0
theta = math.pi/2

vel_x_old = 0
vel_y_old = 0
omega_old = 0

sim_time = 0
# old_time = time.time()
new_time = 0
first_time = True
# time_zero = time.time() #TODO
time_zero = 0

#odom_time-Branch #TODO
odom_time_start = 0 
odom_time_curr = 0
odom_dt = 0
odom_first_time = True


# ------------------------- call back functions ------------------------- #
def odom_time_callback(msg:Odometry): #odom_time-Branch #TODO
    pass
    global odom_time_curr
    global odom_time_start
    global odom_dt
    global odom_first_time

    if(odom_first_time):
        odom_time_start = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
        odom_first_time = False
    
    odom_time_curr = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9

    odom_dt = odom_time_curr - odom_time_start
    # rospy.loginfo("Time: " + str(odom_dt) )


def clock_callback(msg:Clock):
    pass
    global sim_time

    sim_time = msg.clock
    sim_time = sim_time.secs + sim_time.nsecs * 1e-9

    # sim_time = time.time()

def odom_callback(msg:Odometry):
    pass
    # Position Initialization:
    global x
    global y
    global z

    pose = msg.pose.pose

    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    # Velocities Initialization:
    global vel_x_old
    global vel_y_old
    global omega_old

    twist = msg.twist.twist

    vel_x_old = twist.linear.x
    vel_y_old = twist.linear.y
    omega_old = twist.angular.z

    odom_subscriber.unregister()

def encoder_callback(msg:Float64MultiArray):
    pass
    # rospy.loginfo(msg.data)

    r_wheel = 0.340175
    d = 0.82491 # 2*d = track width (distance between two back wheels)
    # d = 0.8
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

# ------------------------- Regular Functions ------------------------- #
def integrate(vel, omega):
    pass
    global x
    global y
    global theta
    global old_time
    global new_time
    global first_time
    global time_zero
    global vel_x_old
    global vel_y_old
    global omega_old

    # global test_dt #TODO remove

    if(first_time):
        # old_time = time.time()
        old_time = sim_time #TODO
        time_zero = old_time
        first_time = False

    # new_time = time.time()
    new_time = sim_time #TODO

    dt = new_time - old_time
    # dt = 1/60

    vel_x = vel * math.cos(theta) 
    vel_y = vel * math.sin(theta)

    # -------------------  Euler ----------------------- #
    # x = x + vel_x*dt
    # y = y + vel_y*dt
    # theta = theta + omega*dt

    # -------------------  Trapazoid ----------------------- #
    x = x + ( (vel_x + vel_x_old)/2 )*dt
    y = y + ( (vel_y + vel_y_old)/2 )*dt
    theta = theta + ( (omega + omega_old)/2 )*dt

    vel_x_old = vel_x
    vel_y_old = vel_y
    omega_old = omega

    # -------------------  RK4 ----------------------- #

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
    out.z = theta_out #TODO uncomment

    # if(dt == 0): #TODO remove
    #     out.z = 0
    # else:
    #     out.z = 1/dt

    # out.z = elapsed #TODO remove

    # out.z = odom_dt #TODO remove

    vehicle_position_publisher.publish(out)


    
# ------------------------- Main ------------------------- #
if __name__ == "__main__":
    # ------------------------- Global Variables ------------------------- #
    

    # ------------------------- Node initiation ------------------------- #
    rospy.loginfo("encoder_se Start")
    rospy.init_node("encoder_se")


    # ------------------------- Topics ------------------------- #
    wheel_vel_topic = ("/wheel_vel" , Float64MultiArray)
    state_estimation_encoder_topic = ('/vehicle_velocities', Vector3)
    xyz_estimation_encoder_topic = ('/vehicle_position', Vector3)
    clock_topic = ('/clock', Clock)
    odom_topic = ('/odom', Odometry)

    # ------------------------- Subscribers ------------------------- #
    
    clock_subscriber = rospy.Subscriber(clock_topic[0], clock_topic[1] , callback=clock_callback)
    rospy.wait_for_message(clock_topic[0], clock_topic[1]) # Manually call clock_callback to initialize sim_time

    odom_subscriber = rospy.Subscriber(odom_topic[0], odom_topic[1] , callback=odom_callback)
    odom_time_subscriber = rospy.Subscriber(odom_topic[0], odom_topic[1] , callback=odom_time_callback) #odom_time-Branch #TODO remove
    rospy.wait_for_message(odom_topic[0], odom_topic[1]) # Manually call odom_callback to initialize x,y,z

    wheel_vel_subscriber = rospy.Subscriber(wheel_vel_topic[0], wheel_vel_topic[1], callback=encoder_callback)

    # ------------------------- publishers ------------------------- #

    vehicle_velocities_publisher = rospy.Publisher(state_estimation_encoder_topic[0], state_estimation_encoder_topic[1], queue_size=10)
    vehicle_position_publisher = rospy.Publisher(xyz_estimation_encoder_topic[0], xyz_estimation_encoder_topic[1], queue_size=10)

    # ------------------------- Rest of Code ------------------------- #
    rospy.spin()

    rospy.loginfo("encoder_se Exit")

    duration = new_time - time_zero
    rospy.loginfo("Duration: " + str(duration) )