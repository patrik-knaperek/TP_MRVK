#!/usr/bin/env python

import math
import time

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class Controller(object):
    def __init__(self, pub, PT, PR, ref):
        self.pub = pub
        self.PT = PT
        self.PR = PR
        self.ref = ref
        self.v = 0.0
        self.w = 0.0
        self.init = False
        self.timer_init = False
        self.timer_start = time.time()
        self.timer_end = time.time()
        self.dt = 0.0

    def timer(self):
        if (self.timer_init):
            self.timer_end = time.time()
            self.dt = self.timer_end - self.timer_start
            self.timer_start = self.timer_end
        else:
            self.timer_start = time.time()
            self.timer_init = True

    def rate_limiter(self, y, u, rising_slew_rate):
        rate = (u - y)/self.dt

        if (rate > rising_slew_rate):
            return y + rising_slew_rate*self.dt

        return u

    def actuating_signal(self, x, y, ang):
        self.timer()
        dx = self.ref[0] - x
        dy = self.ref[1] - y
        e = math.sqrt(dx**2 + dy**2)
        r = math.atan2(dy, dx) - ang

        if (not self.init):
            self.init = True
            return None

        if (e < 0.1):
            self.v = 0
            self.w = 0
        else:
            tmp = self.PT*e
            self.v = self.rate_limiter(self.v, tmp, 0.1)
            tmp = self.PR*r
            self.w = self.rate_limiter(self.w, tmp, 0.1)

    def odometry_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        ang = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])

        print(ang)
        rospy.loginfo('x: {}, y: {}, ang: {}'.format(x, y, ang))

        self.actuating_signal(x, y, ang[2])

        msg = Twist()
        msg.linear.x = self.v
        msg.angular.z = self.w
        #self.pub.publish(msg)



def main():
    ref = (0, 0)

    rospy.init_node('position_control')
    pup = rospy.Publisher('/mrvk_diff_drive_controller/cmd_vel', Twist, queue_size = 10)
    controller = Controller(pup, 1, 1, ref)
    rospy.Subscriber("/test_robot/odom", Odometry, controller.odometry_callback)
    rospy.spin()

if __name__ == '__main__':
    main()