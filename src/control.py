#!/usr/bin/env python
import rospy
import rosservice
import tf2_ros
import tf2_msgs.msg

from geometry_msgs.msg import Vector3, Twist, TransformStamped
from geometry_msgs.msg import Twist
from decentralized_search.srv import VoxelUpdate, GoalUpdate, Tolerance
from std_msgs.msg import Int8
import numpy as np
from math import *


class AgentNode(object):
    tol = None

    def __init__(self, robot_id, initial_x, initial_y):
        self.id = robot_id
        self.request_new_location = rospy.ServiceProxy("/voxel_update", VoxelUpdate)
        self.pub = rospy.Publisher("/robot" + str(self.id) + "/cmd_vel", Twist, queue_size=10)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.service_goal = rospy.ServiceProxy("/goals", GoalUpdate)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.r = rospy.Rate(50)

        self.K1 = 1
        self.K2 = 3
        self.curr_x = initial_x
        self.curr_y = initial_y

        tol_service = rospy.ServiceProxy("/tolerance", Tolerance)
        rospy.wait_for_service("/tolerance")
        AgentNode.tol = tol_service().tolerance
        rospy.Subscriber("/finished", Int8, self.shutdown)

        self.first = True
        before_wait = {0: 0, 1: 5, 2: 10, 3: 0}
        self.after_wait = {0: 13, 1: 10, 2: 5, 3: 15}
        rospy.wait_for_service("/voxel_update")
        rospy.sleep(before_wait[self.id])
        self.run_control_loop()

    def shutdown(self, msg):
        rospy.signal_shutdown('done')

    def run_control_loop(self):
        while not rospy.is_shutdown():
            rospy.wait_for_service("/voxel_update")
            try:
                response = self.request_new_location(self.curr_x, self.curr_y, self.id)
                rospy.sleep(self.after_wait[self.id]) if self.first else None
                self.first = False
                self.move_to_location(response.x, response.y)
            except rospy.service.ServiceException:
                pass

    def move_to_location(self, x, y):
        self.publish_tf(self.id, x, y)
        rospy.sleep(.1)
        self.move_to_frame("robot" + str(self.id), x, y)
        self.curr_x, self.curr_y = x, y

    def move_to_frame(self, robot_frame, x, y):
        while not rospy.is_shutdown():
            try:
                # fromWorld = self.tfBuffer.lookup_transform("map_static", robot_frame, rospy.Time(0))
                # heading = fromWorld.transform.rotation.z

                self.publish_tf(self.id, x, y)

                dt = self.tfBuffer.lookup_transform(robot_frame, "target" + str(self.id),
                                                    rospy.Time(0)).transform.translation

                # if heading > pi:
                #     heading -= 2 * pi
                #
                # dx = x - fromWorld.transform.translation.x
                # dy = y - fromWorld.transform.translation.y
                # r = sqrt(dx ** 2 + dy ** 2)
                #
                # # ### START AUSTIN'S CODE
                # bot_vector_x = np.array([np.cos(heading), np.sin(heading)])
                # bot_vector_y = np.array([np.cos(heading + pi/2), np.sin(heading + pi/2)])
                #
                # target_vector = np.array([dx, dy])# / r

                rel_x = dt.x#self.project(bot_vector_x, target_vector)
                rel_y = dt.y#self.project(bot_vector_y, target_vector)
                r = sqrt(dt.x ** 2 + dt.y ** 2)

                # if self.id == 0:
                #     print("-----------")
                #     print([dt.x, dt.y])
                #     print([rel_x, rel_y])

                """if robot_frame == 'robot0':
                    print(target_vector)
                    print(rel_x, rel_y)"""

                ### END AUSTIN'S CODE
                """heading = heading % (2 * pi)
                theta = pi
                if dx == 0:
                    theta = pi / 2 if dy > 0 else -pi / 2
                elif dx > 0:
                    theta = atan(dy / dx)
                else:
                    theta += atan(dy / dx)
                dtheta = heading - theta
                dtheta = dtheta % (2 * pi)
                if dtheta > pi:
                    dtheta -= 2 * pi
                # rel_x = r * cos(dtheta)
                # rel_y = -r * sin(dtheta)
                if robot_frame == "robot0":
                    print([fromWorld.transform.translation.x, fromWorld.transform.translation.y, dtheta])"""
                control_command = Twist()
                if r < AgentNode.tol ** 2:
                    self.pub.publish(control_command)
                    return
                control_command.linear.x = self.K1 * rel_x
                control_command.angular.z = self.K2 * rel_y

                self.pub.publish(control_command)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, rospy.exceptions.ROSException):
                pass
            self.r.sleep()

    def project(self, vector1, vector2):
        return np.sum(vector1 * vector2)

    def publish_tf(self, id, x, y):
        t = TransformStamped()
        t.header.frame_id = "map_static"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "target" + str(id)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    bot_id = rospy.get_param("~robot_id")
    init_x, init_y = rospy.get_param("~x"), rospy.get_param("~y")
    AgentNode(bot_id, init_x, init_y)
    rospy.spin()

