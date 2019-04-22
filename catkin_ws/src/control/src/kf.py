#!/usr/bin/env python
import rospy
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import TransformStamped
from math import pow, atan2, sqrt, pi, degrees
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time
import numpy as np

def euclidean_distance(xd, yd, zd):
    return sqrt(pow((xd), 2) + pow((yd), 2) + pow((zd), 2))


class State:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x = x
        self.y = y
	self.z = z

class PID:
    def __init__(self,kp=1,kd=0,ki=0,dt=0.01):

        #GAINS
        self.kp = kp
        self.kd = kd
        self.ki = ki

        #TIME STEP
        self.dt = dt

        #Default ERROR INITIALIZATION
        self.err_previous = 0.001
        self.err_acc = 0

    def compute(self,err):

        #compute dervivative
        err_deriv = (err - self.err_previous)/self.dt
        
        #update integration
        self.err_acc = self.err_acc + self.dt * (err + self.err_previous)/2
        
        #compute pid equation
        pid = self.kp*err + self.kd*err_deriv + self.ki*self.err_acc

        #update error
        self.err_previous = err

        return pid

class Controller:
    '''
    main class of the ros node controlling the robot.
    '''

    def __init__(self):

        #initialization of the ros node and relevant pub/sub
        rospy.init_node("PID_node")
        self.velocity_publisher=rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped",Twist,queue_size=1)
        self.position_publisher=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=1) 
        self.bebop_subscriber=rospy.Subscriber("/relative_distance", Float32MultiArray ,self.call_back)
        self.position_subscriber=rospy.Subscriber("/mavros/local_position/pose", PoseStamped ,self.pos_call_back)

        #robot current state
        self.state = State()
        self.local_position = PoseStamped()
        self.velocity = State()

        #controller frequency in Hz
        self.hz=50.0
        self.rate = rospy.Rate(self.hz)
        self.dt=(1.0/self.hz)

        #define pids
        self.pid_rho_xy = PID(kp=0.3, ki=0.02, dt=self.dt)

        #kf parameters
        self.A = np.eye(2, dtype = float)
        self.B = -self.dt * np.eye(2, dtype = float)
        self.C = np.eye(2, dtype = float)
        self.Q = 0.05 * np.eye(2, dtype = float)
        self.R = 0.05 * np.eye(2, dtype = float)
        self.P = np.eye(2, dtype = float)

    def kf_predict(self, X, U):
        # predict state and covariance
        pred_X = np.dot(self.A, X) + np.dot(self.B, U)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return pred_X

    def kf_update(self, pred_X, Y):
        # update state
        S = self.R + np.dot(self.C, np.dot(self.P, self.C.T))
        K = np.dot(np.dot(self.P, self.C.T), np.linalg.inv(S))
        corr_X = pred_X + np.dot(K, Y - np.dot(self.C, pred_X))

        # update state and covariance
        I = np.eye(2)
        self.P = np.dot(I - np.dot(K, self.C), self.P)

        return corr_X.reshape(2, )

    # transform from camera frame to drone frame
    def call_back(self, msg):

        # prediction
        pred_X = None
        if self.velocity.x is not 0.0 and not math.isnan(self.state.x):
            U = np.array([self.velocity.x, self.velocity.y]).reshape(2, 1)
            X = np.array([self.state.x, self.state.y]).reshape(2, 1)
            pred_X = self.kf_predict(X, U)
        
        # sensor(camera) measurement
        self.state.x = -msg.data[1]
        self.state.y = -msg.data[0]
        self.state.z = -msg.data[2]

        # state estimation(only xy)
        if pred_X is not None:
            Y = np.array([self.state.x, self.state.y]).reshape(2, 1)
            corr_X = self.kf_update(pred_X, Y)
            self.state.x = corr_X[0]
            self.state.y = corr_X[1]


    def pos_call_back(self, msg):
        self.local_position.pose.position.x = msg.pose.position.x
        self.local_position.pose.position.y = msg.pose.position.y
        self.local_position.pose.position.z = msg.pose.position.z


    def takeoff(self):
        # Arm
        #print "\nArming"
        #rospy.wait_for_service('/mavros/cmd/arming')
        #try:
        #    arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        #    response = arming_cl(value = True)
        #    rospy.loginfo(response)
        # except rospy.ServiceException as e:
        #     print("Arming failed: %s" %e)

        # # Takeoff
        # print "\nTaking off"
        # rospy.wait_for_service('/mavros/cmd/takeoff')
        # try:
        #     takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        #     response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
        #     rospy.loginfo(response)
        # except rospy.ServiceException as e:
        #     print("Takeoff failed: %s" %e)
        # print "\n Hovering..."
        # time.sleep(3)
        
        print "\n Takeoff..."
        pos_msg = PoseStamped()
        pos_msg.pose.position.x = 0
        pos_msg.pose.position.y = 0
        pos_msg.pose.position.z = 2
        print("Fly to z = {}".format(pos_msg.pose.position.z))
        for i in range(200):
            self.position_publisher.publish(pos_msg)
            self.rate.sleep()
        
    def move_to_goal(self):
        #variable initialization
        vel_msg = Twist()
        pose_msg = PoseStamped()
        tolerance_distance_xy = 0.01

        rho_xy = euclidean_distance(self.state.x, self.state.y, 0.0)
        # print("rho: {}".format(rho))

        while (rho_xy >= tolerance_distance_xy or rho_xy == 0) and not math.isnan(self.state.x):
            rospy.loginfo("xy Distance from goal:"+str(rho_xy))
            rho_xy = euclidean_distance(self.state.x, self.state.y, 0.0)

            # pid control
            vx = self.pid_rho_xy.compute(self.state.x)
            vy = self.pid_rho_xy.compute(self.state.y)

            if (rho_xy / abs(self.state.z)) >= 0.1:
                vz = 0.0
            else:
                vz = -0.25

            #fill message
            vel_msg.linear.x = vx
            vel_msg.linear.y = vy
            vel_msg.linear.z = vz
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            #debugging
            print("vx: {:6f}, x distance: {:6f}".format(vel_msg.linear.x, self.state.x))
            print("vy: {:6f}, y distance: {:6f}".format(vel_msg.linear.y, self.state.y))
            print("vz: {:6f}, z distance: {:6f}".format(vel_msg.linear.z, self.state.z))
            print("_________________")

            #publish
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # stop the robot
        vel_msg.linear.x = 0.0
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        self.velocity_publisher.publish(vel_msg)

        # Land
        print "\n Landing"
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            land_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            response = land_cl(altitude = abs(self.state.z), latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Landing failed: %s" %e)

        # Disarm
        print "\n Disarming"
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = False)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Disarming failed: %s" %e)

        rospy.loginfo("I'm here(relative info): "+ str(self.state.x) + " , " + str(self.state.y) +" , " + str(self.state.z))
        print("___")

        return


if __name__ == '__main__':
    try:
        drone = Controller()

        # TAKE OFF
        drone.takeoff()

        # MOVE TO THE GOALS
        drone.move_to_goal()

        # SPIN
        rospy.spin()

    except rospy.ROSInterruptException:
        pass 
