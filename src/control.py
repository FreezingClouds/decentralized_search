#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg

from geometry_msgs.msg import Vector3, Twist, TransformStamped
from geometry_msgs.msg import Twist
from decentralized_search.srv import VoxelUpdate, GoalUpdate

tol = 0.15  # Should be less than the radius

class AgentNode(object):
    def __init__(self, robot_id, initial_x, initial_y):
        self.id = robot_id
        self.request_new_location = rospy.ServiceProxy("/voxel_update", VoxelUpdate)
        self.pub = rospy.Publisher("/robot" + str(self.id) + "/cmd_vel", Twist, queue_size=10)
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.service_goal = rospy.ServiceProxy("/goals", GoalUpdate)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.r = rospy.Rate(10)
        self.K1 = 0.5
        self.K2 = 1
        self.curr_target_location = None
        self.curr_x = initial_x
        self.curr_y = initial_y
        self.run_control_loop()

    def run_control_loop(self):
        while not rospy.is_shutdown():
            rospy.wait_for_service("/voxel_update")
            response = self.request_new_location(self.curr_x, self.curr_y, self.id)
            self.move_to_location(response.x, response.y)

    def move_to_location(self, x, y):
        rospy.wait_for_service("/goals")
        response = self.service_goal(x, y, self.id)
        if response.received:
            self.move_to_frame("robot" + str(self.id), "target" + str(self.id))
            self.curr_x, self.curr_y = x, y

    def move_to_frame(self, robot_frame, target_frame):
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time(0))
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
    rospy.init_node('dummy', anonymous=True)
    bot_id = rospy.get_param("~robot_id")
    init_x, init_y = rospy.get_param("~x"), rospy.get_param("~y")
    AgentNode(bot_id, init_x, init_y)
    rospy.spin()

