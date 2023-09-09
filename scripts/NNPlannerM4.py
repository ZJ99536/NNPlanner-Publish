from numpy import loadtxt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Normalization
from tensorflow import keras
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import rospy
from std_msgs.msg import  Bool
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import PoseStamped,Vector3Stamped,TwistStamped
import numpy as np
from time import time
from sensor_msgs.msg import Imu

current_position = np.zeros(3)
current_velocity = np.zeros(3)
last_velocity = np.zeros(3)
current_acc = np.zeros(3)
current_acc[2] = 9.81
current_rate = np.zeros(3)
is_ready = 0

ros_freq = 33.0

last_time = time()

last_pos_cmd = np.zeros((2,3))
last_vel_cmd = np.zeros((2,3))
last_acc_cmd = np.zeros((2,3))
last_pqr_cmd = np.zeros((2,3))

flag = 1

kx = np.zeros(2)
ky = np.zeros(2)
kz = np.zeros(2)
kvx = np.zeros(2)
kvy = np.zeros(2)
kvz = np.zeros(2)
kax = np.zeros(2)
kay = np.zeros(2)
kaz = np.zeros(2)
kqx = np.zeros(2)
kqy = np.zeros(2)
kqz = np.zeros(2)
 
kx = [0.2,0.3,0.5]
kx = [0.1,0.3,0.6] #8
# ky = [0.2,0.3,0.5] #1
# kz = [0.2,0.3,0.5] #1
ky = [0.1,0.3,0.6] #2
kz = [0.1,0.3,0.6] #2
# ky = [0.15,0.3,0.55] #9
# kz = [0.1,0.3,0.6] #9
# ky = [0.0,0.0,1.0] #3
# kz = [0.0,0.0,1.0] #3
# ky = [0.1,0.1,0.8] #4
# kz = [0.1,0.1,0.8] #4
# kvx = [0.1,0.3,0.6]
# kvy = [0.1,0.3,0.6]
# kvz = [0.1,0.3,0.6]
kvx = [0.2,0.3,0.5]
kvx = [0.1,0.3,0.6] #8
# kvy = [0.2,0.3,0.5]
# kvz = [0.2,0.3,0.5]
kvy = [0.1,0.3,0.6] #2
kvz = [0.1,0.3,0.6] #2
# kvy = [0.0,0.0,1.0] #3
# kvz = [0.0,0.0,1.0] #3
# kvy = [0.1,0.1,0.8] #4
# kvz = [0.1,0.1,0.8] #4
kax = [0.2,0.4,0.4]
kay = [0.2,0.4,0.4]
kaz = [0.2,0.4,0.4]
# kax = [0.1,0.3,0.6]
# kay = [0.3,0.3,0.4]
# kaz = [0.3,0.3,0.4] #12
kqx = [0.2,0.4,0.4]
kqy = [0.2,0.4,0.4]
kqz = [0.2,0.4,0.4]

kx = [0.0,0.0,1.0]
ky = [0.0,0.0,1.0]
kz = [0.0,0.0,1.0]
kvx = [0.0,0.0,1.0]
kvy = [0.0,0.0,1.0]
kvz = [0.0,0.0,1.0]
kax = [0.0,0.0,1.0]
kay = [0.0,0.0,1.0]
kaz = [0.0,0.0,1.0]
kqx = [0.0,0.0,1.0]
kqy = [0.0,0.0,1.0]
kqz = [0.0,0.0,1.0]



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



# def odometry_cb(data):
#     global current_position
#     current_position = np.array([data.pose.pose.position.x,data.pose.pose.position.y,data.pose.pose.position.z])
#     global current_velocity
#     current_velocity = np.array([data.twist.twist.linear.x,data.twist.twist.linear.y,data.twist.twist.linear.z])
#     global current_rate
#     current_rate = np.array([data.twist.twist.angular.x,data.twist.twist.angular.y,data.twist.twist.angular.z])
#     global current_acc
#     global last_velocity
#     current_acc = (current_velocity - last_velocity) * ros_freq
#     current_acc[2] += 9.81
#     last_velocity = current_velocity
p_init = 0
v_init = 0
a_init = 0
q_init = 0
p_flag = 1
v_flag = 1
a_flag = 1
q_flag = 1
p_queue = []
v_queue = []
a_queue = []
q_queue = []

def position_cb(data):
    global current_position
    global p_queue
    global p_flag
    global p_init
    p_queue.append(data)
    if p_flag:
        p_init = rospy.Time.now()
        p_flag = 0
    time_now = (rospy.Time.now() - p_init).to_sec()
    if time_now > 0.00:
        data = p_queue[0]
        p_queue.pop(0)
        current_position = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
   
def velocity_cb(data):
    global current_velocity
    global v_queue
    global v_flag
    global v_init
    v_queue.append(data)
    if v_flag:
        v_init = rospy.Time.now()
        v_flag = 0
    time_now = (rospy.Time.now() - v_init).to_sec()
    if time_now > 0.00:
        data = v_queue[0]
        v_queue.pop(0)   
        current_velocity = np.array([data.twist.linear.x,data.twist.linear.y,data.twist.linear.z])

def acc_cb(data):
    global current_acc
    global a_queue
    global a_flag
    global a_init
    a_queue.append(data)
    if a_flag:
        a_init = rospy.Time.now()
        a_flag = 0
    time_now = (rospy.Time.now() - a_init).to_sec()
    if time_now > 0.00:
        data = a_queue[0]
        a_queue.pop(0) 
    current_acc = np.array([data.twist.linear.x,data.twist.linear.y,data.twist.linear.z])

def rate_cb(data):
    global current_rate
    global q_queue
    global q_flag
    global q_init
    q_queue.append(data)
    if q_flag:
        q_init = rospy.Time.now()
        q_flag = 0
    time_now = (rospy.Time.now() - q_init).to_sec()
    if time_now > 0.00:
        data = q_queue[0]
        q_queue.pop(0) 
    current_rate = np.array([data.angular_velocity.x, -data.angular_velocity.y, -data.angular_velocity.z])

def status_cb(data):
    global is_ready
    is_ready = data.data

    
if __name__ == "__main__":

    waypoint0 = np.array([0.0, 0.5, 1.3])
    waypoint1 = np.array([2.0, 0.0, 1.0])
    current_lambda = [1, 1]

    rospy.init_node("planner")

    # odom_sub = rospy.Subscriber('/uav0/mavros/local_position/odom',Odometry,odometry_cb)
    position_sub = rospy.Subscriber('/outer_position', PoseStamped,position_cb)
    velocity_sub = rospy.Subscriber('/outer_velocity', TwistStamped,velocity_cb)
    acc_sub = rospy.Subscriber('/outer_acc', TwistStamped,acc_cb)
    rate_sub = rospy.Subscriber('/mavros/imu/data', Imu, rate_cb)
    # position_sub = rospy.Subscriber('/outer_position', PoseStamped,position_cb)
    status_sub = rospy.Subscriber('/offb_node/status',Bool,status_cb)

    position_planner_pub = rospy.Publisher('planner/position', Vector3Stamped, queue_size=1)
    position_setpoint = Vector3Stamped()
    velocity_planner_pub = rospy.Publisher('planner/velocity', Vector3Stamped, queue_size=1)
    velocity_setpoint = Vector3Stamped()
    attitude_planner_pub = rospy.Publisher('planner/attitude', Vector3Stamped, queue_size=1)
    attitude_setpoint = Vector3Stamped()
    rate_planner_pub = rospy.Publisher('planner/rate', Vector3Stamped, queue_size=1)
    rate_setpoint = Vector3Stamped()
    status_planner_pub = rospy.Publisher('planner/status', Vector3Stamped, queue_size=1)
    status_setpoint = Vector3Stamped()


    rate = rospy.Rate(ros_freq)

    # model = keras.models.load_model('/home/zhoujin/learning/model/quad5_t3.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quad2_5t2.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quad4_75t2.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quadback_75t2.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quad_FB.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quad5_m5.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model = keras.models.load_model('/home/zhoujin/learning/model/quad2_ALL.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model0 = keras.models.load_model('/home/zhoujin/learning/model/quad2_8m.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    # model1 = keras.models.load_model('/home/zhoujin/learning/model/quad2_8m.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    model0 = keras.models.load_model('/home/zhoujin/learning/model/quad_FB.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    model1 = keras.models.load_model('/home/zhoujin/learning/model/quadm2_75t2.h5') # quad5 m4 m6(softplus 64) m5(softplus 640)
    status = 0
    while not rospy.is_shutdown():
        if status == 0 :
            model = model0
            waypoint0 = np.array([0.0, -0.8, 1.3])
            waypoint1 = np.array([2.0, -0.4, 1.0])
            error = waypoint1 - current_position
            if error[0]**2 + error[1]**2 + error[2]**2 < 0.15:
                status = 1
        if status == 1 :
            model = model1
            waypoint0 = np.array([0.0, 0.0, 1.3])
            waypoint1 = np.array([2.0, 0.4, 1.0])
            error = waypoint1 - current_position
            if error[0]**2 + error[1]**2 + error[2]**2 < 0.4:
                status = 2
        if status == 2 :
            model = model0
            waypoint0 = np.array([0.0, 0.8, 1.3])
            waypoint1 = np.array([-2.0, 0.4, 1.0])   

        status_setpoint.vector.x = status
        status_setpoint.header.stamp = rospy.Time.now()
        status_planner_pub.publish(status_setpoint)

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
            position_setpoint.vector.x = (((waypoint0[0] - output[0, 0]) + (waypoint1[0] - output[0, 3])) / 2) * kx[len(kx)-1]
            position_setpoint.vector.y = (((waypoint0[1] - output[0, 1]) + (waypoint1[1] - output[0, 4])) / 2) * ky[len(kx)-1]
            # if (((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2) > 0:
            position_setpoint.vector.z = (((waypoint0[2] - output[0, 2]) + (waypoint1[2] - output[0, 5])) / 2) * kz[len(kx)-1]
            
            for i in range(len(kx)-1):
                position_setpoint.vector.x += last_pos_cmd[i,0] * kx[i] 
                position_setpoint.vector.y += last_pos_cmd[i,1] * ky[i] 
                position_setpoint.vector.z += last_pos_cmd[i,2] * kz[i] 
            position_setpoint.header.stamp = rospy.Time.now()
            position_planner_pub.publish(position_setpoint)

            velocity_setpoint.vector.x = output[0, 6] * kvx[len(kvx)-1]
            velocity_setpoint.vector.y = output[0, 7] * kvy[len(kvx)-1]
            velocity_setpoint.vector.z = output[0, 8] * kvz[len(kvx)-1]
            for i in range(len(kvx)-1):
                velocity_setpoint.vector.x += last_vel_cmd[i,0] * kvx[i] 
                velocity_setpoint.vector.y += last_vel_cmd[i,1] * kvy[i] 
                velocity_setpoint.vector.z += last_vel_cmd[i,2] * kvz[i] 
            velocity_setpoint.header.stamp = rospy.Time.now()
            velocity_planner_pub.publish(velocity_setpoint)

            attitude_setpoint.vector.x = output[0, 9] * kax[len(kax)-1]
            attitude_setpoint.vector.y = output[0, 10] * kay[len(kax)-1]
            attitude_setpoint.vector.z = output[0, 11] * kaz[len(kax)-1]
            for i in range(len(kax)-1):
                attitude_setpoint.vector.x += last_acc_cmd[i,0] * kax[i] 
                attitude_setpoint.vector.y += last_acc_cmd[i,1] * kay[i] 
                attitude_setpoint.vector.z += last_acc_cmd[i,2] * kaz[i] 
            attitude_setpoint.header.stamp = rospy.Time.now()
            attitude_planner_pub.publish(attitude_setpoint)


            rate_setpoint.vector.x = output[0, 12] * kqx[len(kqx)-1]
            rate_setpoint.vector.y = output[0, 13] * kqy[len(kqx)-1]
            rate_setpoint.vector.z = output[0, 14] * kqz[len(kqx)-1]
            for i in range(len(kqx)-1):
                rate_setpoint.vector.x += last_pqr_cmd[i,0] * kqx[i] 
                rate_setpoint.vector.y += last_pqr_cmd[i,1] * kqy[i] 
                rate_setpoint.vector.z += last_pqr_cmd[i,2] * kqz[i] 
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

        for i in range(len(kx)-2):
            last_pos_cmd[i,0] = last_pos_cmd[i+1,0]
            last_pos_cmd[i,1] = last_pos_cmd[i+1,1]
            last_pos_cmd[i,2] = last_pos_cmd[i+1,2]
            last_vel_cmd[i,0] = last_vel_cmd[i+1,0]
            last_vel_cmd[i,1] = last_vel_cmd[i+1,1]
            last_vel_cmd[i,2] = last_vel_cmd[i+1,2]
            last_acc_cmd[i,0] = last_acc_cmd[i+1,0]
            last_acc_cmd[i,1] = last_acc_cmd[i+1,1]
            last_acc_cmd[i,2] = last_acc_cmd[i+1,2]
            last_pqr_cmd[i,0] = last_pqr_cmd[i+1,0]
            last_pqr_cmd[i,1] = last_pqr_cmd[i+1,1]
            last_pqr_cmd[i,2] = last_pqr_cmd[i+1,2]

        last_pos_cmd[len(kx)-2,0] = position_setpoint.vector.x
        last_pos_cmd[len(kx)-2,1] = position_setpoint.vector.y
        last_pos_cmd[len(kx)-2,2] = position_setpoint.vector.z
        last_vel_cmd[len(kx)-2,0] = velocity_setpoint.vector.x
        last_vel_cmd[len(kx)-2,1] = velocity_setpoint.vector.y
        last_vel_cmd[len(kx)-2,2] = velocity_setpoint.vector.z
        last_acc_cmd[len(kx)-2,0] = attitude_setpoint.vector.x
        last_acc_cmd[len(kx)-2,1] = attitude_setpoint.vector.y
        last_acc_cmd[len(kx)-2,2] = attitude_setpoint.vector.z
        last_pqr_cmd[len(kx)-2,0] = rate_setpoint.vector.x
        last_pqr_cmd[len(kx)-2,1] = rate_setpoint.vector.y
        last_pqr_cmd[len(kx)-2,2] = rate_setpoint.vector.z

        rate.sleep()
