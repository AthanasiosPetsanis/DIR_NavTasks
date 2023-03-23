#!/usr/bin/env python
from math import radians, sqrt, sin, cos
import rospy, numpy as np
from statistics import mean
from geometry_msgs.msg import Vector3
from thanos.General_Functions import now, tic, toc

g = 9.81
k = np.zeros(3)
b = 0.0
er = 0
w = [0, 0, 0]
nof_samples = 51

data_accel = np.zeros((nof_samples+1,3))
accel_error = np.zeros(3)
stop = False
stop2 = False

angles = [0, 0, 0]
accel = [0, 0, 0]
start = 0; total_dt = 0 

t = -1
cur_w = [0, 0, 0]; old_angles = [0, 0, 0]; w = [0, 0, 0]; total_w = [0, 0, 0]; 

def get_angles(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x)
    global angles
    angles[0] = data.x
    angles[1] = data.y
    angles[2] = data.z

def get_accel(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x)
    global accel
    accel[0] = data.x
    accel[1] = data.y
    accel[2] = data.z

def calculate_error(data):
    global accel_error, k
    accel_error[0] = mean(data[:,0])
    accel_error[1] = mean(data[:,1])
    accel_error[2] = mean(data[:,2]) - g
    k[2] = sqrt(accel_error[0]**2+accel_error[1]**2)
    k[1] = sqrt(accel_error[0]**2+accel_error[2]**2)
    k[0] = sqrt(accel_error[2]**2+accel_error[1]**2)
    return accel_error

def calibrate_accel(data):
    global accel_error, b
    # accel_error[0] = float(mean(data[:,0]))
    # accel_error[1] = float(mean(data[:,1]))
    accel_error[2] = float(mean(data[:,2]))
    b=accel_error[2]/g
    return b
    
def listener(dt=0.1):
    global t, start, total_w, cur_w, old_angles, angles, accel, total_dt, data_accel, stop, stop2, b, nof_samples, w

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.sleep(dt)
    total_dt += dt
    rad_angles = [0,0,0]
    rad_angles[0] = radians(angles[0])
    rad_angles[1] = radians(angles[1])
    rad_angles[2] = radians(angles[2])

    #  Compute angular velocity
    if(sqrt((accel[2]+g*cos(rad_angles[0]))**2 + (accel[1]+g*sin(rad_angles[0]))**2) >= k[0]):
        w[0] = round((rad_angles[0] - radians(old_angles[0])) / dt,2)
    # if(not(accel[2]+g*cos(rad_angles[0]) > k[0] or accel[2]+g*cos(rad_angles[0] < -k[0] or accel[1]+g*sin(rad_angles[0]) > k[0] or accel[1]+g*sin(rad_angles[0] < -k[0]))):
    else:
        w[0] = 0.0

    if(sqrt((accel[2]+g*cos(rad_angles[1]))**2 + (accel[0]+g*sin(rad_angles[1]))**2) >= k[1]):
        w[1] = round( (rad_angles[1] - radians(old_angles[1])) / dt,2)
    # if(not(accel[2]+g*cos(angles[1]) > k[1] or accel[2]+g*cos(angles[1]) < -k[1] or accel[0]+g*sin(angles[1]) > k[1] or accel[0]+g*sin(angles[1] < -k[1]))):
    else:
        w[1] = 0.0

    if(sqrt(accel[0]**2+accel[1]**2)>=k[2]):
        w[2] = round((rad_angles[2] - radians(old_angles[2])) / dt,2)
        # wz = angles[2] - old_angles[2]
    # if(not(sqrt(accel[0]**2+accel[1]**2)>=k[2])):
    else:
        w[2] = 0.0    

    # print(f'Accel around x: {accel[0]}')
    # print(f'Accel around y: {accel[1]}')
    # cur_w[0] = (angles[0] - old_angles[0]) # Find Difference in value
    # cur_w[1] = (angles[1] - old_angles[1])
    # cur_w[2] = (angles[2] - old_angles[2])

    old_angles[0] = (angles[0])
    old_angles[1] = (angles[1])
    old_angles[2] = (angles[2])
    # # if (wx < 0.5 and wx > -0.5) {wx = 0;} # Define sensitivity
    
    # if cnt < nof_samples:
    #       total_w[0] = (total_w[0] + cur_w[0])
    #     total_w[1] = (total_w[1] + cur_w[1])
    #     total_w[2] = (total_w[2] + cur_w[2])
    #  else:
    #      w[0] = (total_w[0] / nof_samples / dt);# w[1] = (total_w[1] / nof_samples / dt); w[2] = (total_w[2] / nof_samples / dt)
    #     total_dt = 0
    #     total_w[0] = 0; total_w[1] = 0; total_w[2] = 0
    #     cnt = -1
    #     print(w[2])
    # Compute covarience
    if t < nof_samples and not stop:
        t += 1
        data_accel[t,:] = accel
    elif t>=nof_samples and not stop:
        stop = True
        t += 1
        # er = calculate_error(data_accel)
        b = calibrate_accel(data_accel)
    elif t>nof_samples and t<2*nof_samples+2:
        accel = list(np.round(np.divide(accel,b),2))
        data_accel[t-nof_samples-1,:] = accel
        t += 1
    elif not stop2:
        stop2 = True
        # accel = list(np.round(np.divide(accel,b),2))
        calculate_error(data_accel)
    else:
        accel = list(np.round(np.divide(accel,b),2))
        # print(accel)
        print(w)



if __name__ == '__main__':
    start = tic()
    rospy.init_node('arduino_listener', anonymous=True)

    rospy.Subscriber("angle", Vector3, get_angles)
    rospy.Subscriber("acceleration", Vector3, get_accel)
    pub = rospy.Publisher('angular_velocity', Vector3)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        listener()
        pub.publish(w)
    rospy.spin()
