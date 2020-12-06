#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg

from geometry_msgs.msg import Vector3, Twist, TransformStamped
from geometry_msgs.msg import Twist
from decentralized_search.srv import VoxelUpdate, VoxelUpdateRequest

tol = 0.05

class AgentNode(object):
    def __init__(self, robot_id, initial_x, initial_y):
        self.id = robot_id
        self.request_new_location = rospy.ServiceProxy("/voxel_update", VoxelUpdate)
        self.pub = rospy.Publisher("/robot" + str(self.id) + "/cmd_vel", Twist, queue_size=10)
        self.pub_tf = rospy.Publisher("/tf" + str(self.id), tf2_msgs.msg.TFMessage, queue_size=1)
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
            request = VoxelUpdateRequest(self.curr_x, self.curr_y, self.id)
            print()
            response = self.request_new_location(self.curr_x, self.curr_y, self.id)
            self.move_to_location(response.x, response.y)
            self.curr_x, self.curr_y = response.x, response.y

    def move_to_location(self, x, y):
        self.handle_changing_target_frame(x, y)
        rospy.sleep(.1)
        self.move_to_frame(x, y, "robot" + str(self.id), "target" + str(self.id))

    def move_to_frame(self, x, y, robot_frame, target_frame):
        while not rospy.is_shutdown():
            try:
                self.handle_changing_target_frame(x, y)
                trans = self.tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())
                dx = trans.transform.translation.x
                dy = trans.transform.translation.y
                control_command = Twist()
                if dx ** 2 + dy ** 2 < tol ** 2:
                    self.pub.publish(control_command)
                    return
                control_command.linear.x = self.K1 * dx
                control_command.angular.z = self.K2 * dy
                print(control_command.linear.x, control_command.angular.z)

                self.pub.publish(control_command)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                pass
            self.r.sleep()

    def handle_changing_target_frame(self, x, y):
        t = TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "target" + str(self.id)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    bot_id = rospy.get_param("~robot_id")
    init_x, init_y = rospy.get_param("~x"), rospy.get_param("~y")
    AgentNode(bot_id, init_x, init_y)
    rospy.spin()

