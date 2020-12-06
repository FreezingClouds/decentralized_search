#!/usr/bin/env python
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

tol = 0.05

class AgentNode(object):
    def __init__(self, robot_id, initial_x, initial_y):
        self.id = robot_id
        self.request_new_location = rospy.ServiceProxy("/")
        self.pub = rospy.Publisher("/robot" + str(self.id) + "/cmd_vel", Twist, queue_size=10)
        self.pub_tf = rospy.Publisher("/goal_frame" + str(self.id), Vector3, queue_size=10)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.r = rospy.Rate(10)
        self.K1 = 0.3
        self.K2 = 1
        self.curr_target_location = None
        self.curr_x = initial_x
        self.curr_y = initial_y
        self.run_control_loop()

    def run_control_loop(self):
        while not rospy.is_shutdown():
            response = self.request_new_location(x=self.curr_x, y=self.curr_y, id=self.id)
            self.move_to_location(response.x, response.y)
            self.curr_x, self.curr_y = response.x, response.y

    def move_to_location(self, x, y):
        self.pub_tf(Vector3(x, y, 0))
        rospy.sleep(.1)
        self.moveToFrame("robot" + self.id, "target" + self.id)

    def moveToFrame(self, robot_frame, target_frame):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())
                dx = trans.transform.translation.x
                dy = trans.transform.translation.y
                control_command = Twist()
                if dx ** 2 + dy ** 2 < tol ** 2:
                    self.pub.publish(control_command)
                    return
                control_command.linear.x = self.K1 * dx
                control_command.angular.z = self.K2 * dy
                self.pub.publish(control_command)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                pass
            self.r.sleep()


if __name__ == '__main__':
    bot_id = rospy.get_param("~robot_id")
    init_x, init_y = rospy.get_param("~x"), rospy.get_param("~y")
    rospy.init_node('robot' + bot_id, anonymous=True)
    AgentNode(bot_id, init_x, init_y)
    rospy.spin()

