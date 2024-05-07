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
### Car Parameters ###
radius_wheel = 0.340175 # Radius of wheels
track_width = 2*0.82491 # 1.64982 # (distance between two rear wheels)
wheel_base = 2.26972 # distance between front and rear wheels


PI = math.pi
car_poistion_X = 0
car_poistion_Y = 0
car_theta_Z = 0 #TODO math.pi/2 

car_velocity_X = 0
car_velocity_Y = 0
car_velocity_X_old = 0
car_velocity_Y_old = 0
car_omega_Z = 0
car_omega_Z_old = 0

sim_time = 0
new_time = 0
old_time = 0
start_time = 0
elapsed_time = 0
first_time_flag = True


# ------------------------- call back functions ------------------------- #
def odom_time_callback(msg:Odometry): #odom_time-Branch #TODO
    pass


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


def odom_callback(msg:Odometry):
    pass
    # Position Initialization: 
    global car_poistion_X 
    global car_poistion_Y
    global car_theta_Z 

    global car_velocity_X 
    global car_velocity_Y 
    global car_velocity_X_old 
    global car_velocity_Y_old
    global car_omega_Z 
    global car_omega_Z_old 

    pose = msg.pose.pose

    car_poistion_X = pose.position.x
    car_poistion_Y = pose.position.y
    # z = pose.position.z

    # Velocities Initialization:
    global car_velocity_X_old
    global car_velocity_Y_old
    global car_omega_Z_old

    twist = msg.twist.twist

    car_velocity_X_old = twist.linear.x
    car_velocity_Y_old = twist.linear.y
    car_omega_Z_old = twist.angular.z

    odom_subscriber.unregister()

def encoder_callback(msg:Float64MultiArray):
    pass
    ### Global Variables ###   
    global car_poistion_X 
    global car_poistion_Y
    global car_theta_Z 

    global car_velocity_X 
    global car_velocity_Y 
    global car_velocity_X_old 
    global car_velocity_Y_old
    global car_omega_Z 
    global car_omega_Z_old

    global sim_time
    global new_time
    global old_time
    global elapsed_time



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

    ### Car Linear & Angular Velocity ###
    car_velocity = (RL_velocity + RR_velocity)/2
    car_omega_Z =  (RR_velocity - RL_velocity)/track_width

    ### Steering Angle Calculation
    if(car_omega_Z == 0):
        steering_angle = 0
    else:
        I_C_R_distance = car_velocity/car_omega_Z
        steering_angle = math.atan(wheel_base/I_C_R_distance)

    ### Integrate Velocities to find Postion ###
    # integrate()
    ### ------------ Integration START --------------  ###

    new_time = sim_time
    dt = new_time - old_time ##
    old_time = new_time
    
    
    elapsed = new_time - start_time

    # -------------------  Euler ----------------------- #
    # car_poistion_X = car_poistion_X + car_velocity_X*dt
    # car_poistion_Y = car_poistion_Y + car_velocity_Y*dt
    # theta = theta + omega*dt

    # -------------------  Trapazoid ----------------------- #
    car_poistion_X = car_poistion_X + ( (car_velocity_X + car_velocity_X_old)/2 )*dt
    car_poistion_Y = car_poistion_Y + ( (car_velocity_Y + car_velocity_Y_old)/2 )*dt
    car_theta_Z = car_theta_Z + ( (car_omega_Z + car_omega_Z_old)/2 )*dt

    if(car_theta_Z > math.pi):
        car_theta_Z -= 2*math.pi
    elif(car_theta_Z < -math.pi):
        car_theta_Z += 2*math.pi


    #update old values with new values
    car_velocity_X_old = car_velocity_X
    car_velocity_Y_old = car_velocity_Y
    car_omega_Z_old = car_omega_Z

    ### ------------ Integration END --------------  ###

    ### TEST TEST -START- TEST TEST ### #TODO REMOVE


    ### TEST TEST -END- TEST TEST ### #TODO REMOVE


    ### Publish vehicle_velocity Data ###
    ### Linear & Angular Velocities ###
    vehicle_velocity_publishData = Odometry()

    ## Build Data
    car_poistion_X = car_poistion_X
    car_poistion_Y = car_poistion_Y
    car_poistion_Z = 0 

    car_theta_X = 0
    car_theta_Y = 0
    car_theta_Z = car_theta_Z
    car_theta_W = steering_angle #TODO 

    car_velocity_X = car_velocity * math.sin(-car_theta_Z) 
    car_velocity_Y = car_velocity * math.cos(car_theta_Z) 
    car_velocity_Z = 0

    car_omega_X = 0
    car_omega_Y = 0
    car_omega_Z = car_omega_Z

    ## Attach Data
    # Pose
    vehicle_velocity_publishData.pose.pose.position.x = car_poistion_X
    vehicle_velocity_publishData.pose.pose.position.y = car_poistion_Y
    vehicle_velocity_publishData.pose.pose.position.z = car_poistion_Z

    vehicle_velocity_publishData.pose.pose.orientation.x = car_theta_X
    vehicle_velocity_publishData.pose.pose.orientation.y = car_theta_Y
    vehicle_velocity_publishData.pose.pose.orientation.z = car_theta_Z
    vehicle_velocity_publishData.pose.pose.orientation.w = car_theta_W

    # Twist
    vehicle_velocity_publishData.twist.twist.linear.x = car_velocity_X
    vehicle_velocity_publishData.twist.twist.linear.y = car_velocity_Y
    vehicle_velocity_publishData.twist.twist.linear.z = car_velocity_Z

    vehicle_velocity_publishData.twist.twist.angular.x = car_omega_X
    vehicle_velocity_publishData.twist.twist.angular.y = car_omega_Y
    vehicle_velocity_publishData.twist.twist.angular.z = car_omega_Z

    ## Publish Data
    encoder_state_estimation_topic_publisher.publish(vehicle_velocity_publishData)

    # rospy.loginfo("Msg")

# ------------------------- Regular Functions ------------------------- #
def integrate():
    pass
    ### Global Variables ###   
    global car_poistion_X 
    global car_poistion_Y
    global car_theta_Z 

    global car_velocity_X 
    global car_velocity_Y 
    global car_velocity_X_old 
    global car_velocity_Y_old
    global car_omega_Z 
    global car_omega_Z_old

    global sim_time
    global new_time
    global old_time
    global elapsed_time

    ### --------------------------  ###

    new_time = sim_time
    dt = new_time - old_time ##
    old_time = new_time
    
    elapsed = new_time - start_time

    # -------------------  Euler ----------------------- #
    # car_poistion_X = car_poistion_X + car_velocity_X*dt
    # car_poistion_Y = car_poistion_Y + car_velocity_Y*dt
    # theta = theta + omega*dt

    # -------------------  Trapazoid ----------------------- #
    car_poistion_X = car_poistion_X + ( (car_velocity_X + car_velocity_X_old)/2 )*dt
    car_poistion_Y = car_poistion_Y + ( (car_velocity_Y + car_velocity_Y_old)/2 )*dt
    car_theta_Z = car_theta_Z + ( (car_omega_Z + car_omega_Z_old)/2 )*dt

    #update old values with new values
    car_velocity_X_old = car_velocity_X
    car_velocity_Y_old = car_velocity_Y
    car_omega_Z_old = car_omega_Z

    # -------------------  RK4 ----------------------- #


    
# ------------------------- Main ------------------------- #
if __name__ == "__main__":
    # ------------------------- Global Variables ------------------------- #
    

    # ------------------------- Node initiation ------------------------- #
    rospy.loginfo("encoder_se Start")
    rospy.init_node("encoder_se")


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
    odom_time_subscriber = rospy.Subscriber(odom_topic[0], odom_topic[1] , callback=odom_time_callback) #odom_time-Branch #TODO remove
    rospy.wait_for_message(odom_topic[0], odom_topic[1]) # Manually call odom_callback to initialize x,y,z

    wheel_vel_subscriber = rospy.Subscriber(wheel_vel_topic[0], wheel_vel_topic[1], callback=encoder_callback)

    # ------------------------- publishers ------------------------- #

    # vehicle_velocity_publisher = rospy.Publisher(state_estimation_encoder_topic[0], state_estimation_encoder_topic[1], queue_size=10)
    # vehicle_position_publisher = rospy.Publisher(xyz_estimation_encoder_topic[0], xyz_estimation_encoder_topic[1], queue_size=10)
    encoder_state_estimation_topic_publisher = rospy.Publisher(encoder_state_estimation_topic[0], encoder_state_estimation_topic[1], queue_size=10)

    # ------------------------- Rest of Code ------------------------- #
    rospy.spin()

    rospy.loginfo("encoder_se Exit")

    duration = new_time - start_time
    rospy.loginfo("Duration: " + str(duration) )