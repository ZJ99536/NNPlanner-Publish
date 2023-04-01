from numpy import loadtxt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Normalization
from tensorflow import keras
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import rospy
from std_msgs.msg import  Bool
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Vector3Stamped
import numpy as np
from time import time

current_position = np.zeros(3)
current_velocity = np.zeros(3)
last_velocity = np.zeros(3)
current_acc = np.zeros(3)
current_acc[2] = 9.81
current_rate = np.zeros(3)
is_ready = 0

ros_freq = 30.0

last_time = time()

last_pos_cmd = np.zeros(3)
last_vel_cmd = np.zeros(3)
last_acc_cmd = np.zeros(3)
last_pqr_cmd = np.zeros(3)

flag = 1

kx = 0.7
ky = 0.7
kz = 0.7
kvx = 0.6
kvy = 0.6
kvz = 0.6
kax = 0.4
kay = 0.4
kaz = 0.4
kqx = 0.4
kqy = 0.4
kqz = 0.4

#kx = 1
#ky = 1
#kz = 1
#kvx = 1
#kvy = 1
#kvz = 1
#kax = 1
#kay = 1
#kaz = 1
#kqx = 1
#kqy = 1
#kqz = 1



def odometry_cb(data):
    global current_position
    current_position = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
    global current_velocity
    current_velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z])
    global current_rate
    current_rate = np.array([data.twist.twist.angular.x,data.twist.twist.angular.y,data.twist.twist.angular.z])
    global current_acc
    global last_velocity
    global last_time
    current_time = time()
    current_acc = (current_velocity - last_velocity) / (current_time - last_time)
    current_acc[2] += 9.81
    last_velocity = current_velocity
    last_time = current_time


def status_cb(data):
    global is_ready
    is_ready = data.data

    
if __name__ == "__main__":

    waypoint0 = np.array([0.0, 0.5, 1.2])
    waypoint1 = np.array([2.0, 0.0, 1.0])
    current_lambda = [1, 1]

    rospy.init_node("planner")

    odom_sub = rospy.Subscriber('/mavros/local_position/odom',Odometry,odometry_cb)
    status_sub = rospy.Subscriber('/offb_node/status',Bool,status_cb)

    position_planner_pub = rospy.Publisher('planner/position', Vector3Stamped, queue_size=1)
    position_setpoint = Vector3Stamped()
    velocity_planner_pub = rospy.Publisher('planner/velocity', Vector3Stamped, queue_size=1)
    velocity_setpoint = Vector3Stamped()
    attitude_planner_pub = rospy.Publisher('planner/attitude', Vector3Stamped, queue_size=1)
    attitude_setpoint = Vector3Stamped()
    rate_planner_pub = rospy.Publisher('planner/rate', Vector3Stamped, queue_size=1)
    rate_setpoint = Vector3Stamped()


    rate = rospy.Rate(ros_freq)

    model = keras.models.load_model('/home/nesc/planner_ws/src/NNPlanner-Publish/model/quad5_m5.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    
    while not rospy.is_shutdown():

        input = np.zeros(15)
        # print(current_position)
        error0 = waypoint0 - current_position
        error1 = waypoint1 - current_position
        for i in range(3):
            input[i] = error0[i]
            input[i+3] = error1[i]
            input[i+6] = current_velocity[i]
            input[i+9] = current_acc[i]
            input[i+12] = current_rate[i]

        output = model(input.reshape(-1,15))

        if is_ready:
            position_setpoint.vector.x = last_pos_cmd[0] + (((waypoint0[0] - output[0, 0]) + (waypoint1[0] - output[0, 3])) / 2 - last_pos_cmd[0]) * kx
            position_setpoint.vector.y = last_pos_cmd[1] + (((waypoint0[1] - output[0, 1]) + (waypoint1[1] - output[0, 4])) / 2 - last_pos_cmd[1]) * ky
            position_setpoint.vector.z = last_pos_cmd[2] + (((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2 - last_pos_cmd[2]) * kz
            position_setpoint.header.stamp = rospy.Time.now()
            position_planner_pub.publish(position_setpoint)

            velocity_setpoint.vector.x = last_vel_cmd[0] + (output[0, 6] - last_vel_cmd[0]) * kvx
            velocity_setpoint.vector.y = last_vel_cmd[1] + (output[0, 7] - last_vel_cmd[1]) * kvy
            velocity_setpoint.vector.z = last_vel_cmd[2] + (output[0, 8] - last_vel_cmd[2]) * kvz
            velocity_setpoint.header.stamp = rospy.Time.now()
            velocity_planner_pub.publish(velocity_setpoint)

            attitude_setpoint.vector.x = last_acc_cmd[0] + (output[0, 9] - last_acc_cmd[0]) * kax
            attitude_setpoint.vector.y = last_acc_cmd[1] + (output[0, 10] - last_acc_cmd[1]) * kay
            attitude_setpoint.vector.z = last_acc_cmd[2] + (output[0, 11] - last_acc_cmd[2]) * kaz
            attitude_setpoint.header.stamp = rospy.Time.now()
            attitude_planner_pub.publish(attitude_setpoint)


            rate_setpoint.vector.x = last_pqr_cmd[0] + (output[0, 12] - last_pqr_cmd[0]) * kqx
            rate_setpoint.vector.y = last_pqr_cmd[1] + (output[0, 13] - last_pqr_cmd[1]) * kqy
            rate_setpoint.vector.z = last_pqr_cmd[2] + (output[0, 14] - last_pqr_cmd[2]) * kqz
            rate_setpoint.header.stamp = rospy.Time.now()
            rate_planner_pub.publish(rate_setpoint)
        
        else:
            # position_setpoint.vector.x = ((waypoint0[0] - output[0, 0]) + (waypoint1[0] - output[0, 3])) / 2
            position_setpoint.vector.x = current_position[0]
            position_setpoint.vector.y = current_position[1]
            position_setpoint.vector.z = current_position[2]
            # position_setpoint.vector.y = ((waypoint0[1] - output[0, 1]) + (waypoint1[1] - output[0, 4])) / 2
            # position_setpoint.vector.z = ((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2
            position_setpoint.header.stamp = rospy.Time.now()
            position_planner_pub.publish(position_setpoint)

            # velocity_setpoint.vector.x = output[0, 6]
            # velocity_setpoint.vector.y = output[0, 7]
            # velocity_setpoint.vector.z = output[0, 8]
            velocity_setpoint.vector.x = current_velocity[0]
            velocity_setpoint.vector.y = current_velocity[1]
            velocity_setpoint.vector.z = current_velocity[2]
            velocity_setpoint.header.stamp = rospy.Time.now()
            velocity_planner_pub.publish(velocity_setpoint)
            
            # attitude_setpoint.vector.x = output[0, 9]
            # attitude_setpoint.vector.y = output[0, 10]
            # attitude_setpoint.vector.z = output[0, 11]
            attitude_setpoint.vector.x = current_acc[0]
            attitude_setpoint.vector.y = current_acc[1]
            attitude_setpoint.vector.z = current_acc[2]
            attitude_setpoint.header.stamp = rospy.Time.now()
            attitude_planner_pub.publish(attitude_setpoint)


            # rate_setpoint.vector.x = output[0, 12]
            # rate_setpoint.vector.y = output[0, 13]
            # rate_setpoint.vector.z = output[0, 14]
            rate_setpoint.vector.x = current_rate[0]
            rate_setpoint.vector.y = current_rate[1]
            rate_setpoint.vector.z = current_rate[2]
            rate_setpoint.header.stamp = rospy.Time.now()
            rate_planner_pub.publish(rate_setpoint)

        
        last_pos_cmd[0] = position_setpoint.vector.x
        last_pos_cmd[1] = position_setpoint.vector.y
        last_pos_cmd[2] = position_setpoint.vector.z
        last_vel_cmd[0] = velocity_setpoint.vector.x
        last_vel_cmd[1] = velocity_setpoint.vector.y
        last_vel_cmd[2] = velocity_setpoint.vector.z
        last_acc_cmd[0] = attitude_setpoint.vector.x
        last_acc_cmd[1] = attitude_setpoint.vector.y
        last_acc_cmd[2] = attitude_setpoint.vector.z
        last_pqr_cmd[0] = rate_setpoint.vector.x
        last_pqr_cmd[1] = rate_setpoint.vector.y
        last_pqr_cmd[2] = rate_setpoint.vector.z

        rate.sleep()
