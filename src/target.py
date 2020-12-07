#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from decentralized_search.srv import GoalUpdate, GoalUpdateResponse


class FixedTFBroadcaster:
    def __init__(self, num_robots=4):
        self.targets = [(0, 0) for _ in range(num_robots)]
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        rospy.Service("/goals", GoalUpdate, self.receive_new_target)

        num_targets = len(self.targets)
        while not rospy.is_shutdown():
            for i in range(num_targets):
                x, y = self.targets[i]
                self.publish_tf(i, x, y)
            # print(self.targets)

    def publish_tf(self, id, x, y):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "target" + str(id)
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def receive_new_target(self, request):
        x, y, robot_id = request.x, request.y, request.id
        self.targets[robot_id] = (x, y)
        return GoalUpdateResponse(1)


if __name__ == '__main__':
    rospy.init_node('dummy_name', anonymous=True)
    tfb = FixedTFBroadcaster()
    rospy.spin()

