from numpy import loadtxt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Normalization
from tensorflow import keras
# import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import rospy
from std_msgs.msg import  Bool
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistStamped
import numpy as np

current_position = np.zeros(3)
current_velocity = np.zeros(3)
last_velocity = np.zeros(3)
current_acc = np.zeros(3)
current_acc[2] = 9.81
current_rate = np.zeros(3)
is_ready = 0
ros_freq = 10.0
vision_pos = PoseStamped()

def odometry_cb(data):
    global current_position
    current_position = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
    global current_velocity
    current_velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z])
    global current_rate
    current_rate = np.array([data.twist.twist.angular.x,data.twist.twist.angular.y,data.twist.twist.angular.z])
    global current_acc
    global last_velocity
    current_acc = (current_velocity - last_velocity) * ros_freq
    current_acc[2] += 9.81
    last_velocity = current_velocity


def status_cb(data):
    global is_ready
    is_ready = data.data

    
if __name__ == "__main__":

    waypoint0 = np.array([0.0, 0.5, 1.2])
    waypoint1 = np.array([2.0, 0.0, 1.0])
    current_lambda = [1, 1]

    rospy.init_node("planner1")

    odom_sub = rospy.Subscriber('/mavros/local_position/odom',Odometry,odometry_cb)
    
    command_planner_pub = rospy.Publisher('planner1/command', Vector3Stamped, queue_size=1)
    command_setpoint = Vector3Stamped()

    rate = rospy.Rate(ros_freq)

    val = keras.models.load_model('/home/nesc/val_ws/src/NNPlanner-Publish/model/quad5_val.h5')

    while not rospy.is_shutdown():

        input = np.zeros(15)
        input_val = np.zeros(6)
        # print(current_position)
        error0 = waypoint0 - current_position
        error1 = waypoint1 - current_position
        for i in range(3):
            input_val[i] = error0[i]
            input_val[i+3] = error1[i]

        output_val = val(input_val.reshape(-1,6))

        command_setpoint.vector.x = 0
        command_setpoint.vector.y = output_val[0, 0]
        command_setpoint.vector.z = output_val[0, 1]
        command_setpoint.header.stamp = rospy.Time.now()
        command_planner_pub.publish(command_setpoint)
        
        rate.sleep()
