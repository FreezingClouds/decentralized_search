#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

class FixedTFBroadcaster:
    def __init__(self, robot_id):
        self.id = robot_id
        self.x, self.y = None, None
        self.pub_tf = rospy.Publisher("/tf" + str(self.id), tf2_msgs.msg.TFMessage, queue_size=1)
        rospy.Subscriber("/goal_frame" + str(self.id), geometry_msgs.msg.Vector3, self.receive_new_target)

        rospy.wait_for_message("/goal_frame" + str(self.id), geometry_msgs.msg.Vector3)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "target" + self.id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

    def receive_new_target(self, msg):
        self.x, self.y = msg.x, msg.y

if __name__ == '__main__':
    rospy.init_node('dummy_name', anonymous=True)
    robot_id = rospy.get_param('~robot_id')
    tfb = FixedTFBroadcaster(robot_id)
    rospy.spin()

