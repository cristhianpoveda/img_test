#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Twist

class LaneController(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneController, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        self._param = rospy.get_param('~param')

        if self._param == 1:
            rospy.loginfo("si")

        # subscriber to velocity topic
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel)

        # publishers
        self.pub_vel = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=2)

    def cb_cmd_vel(self, duckie_vel):
        
        vel = Twist2DStamped()
        vel.v = duckie_vel.linear.x
        vel.omega = duckie_vel.angular.z

        self.pub_vel.publish(vel)

if __name__ == '__main__':
    node = LaneController(node_name='node')
    rospy.spin()