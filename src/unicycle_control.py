#!/usr/bin/env python
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

tol = 0.05

def moveTo(robot_frame, target_frame):
  pub = rospy.Publisher("/" + robot_frame + "/cmd_vel", Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  r = rospy.Rate(10)

  K1 = 0.3
  K2 = 1
  i = 0
  while not rospy.is_shutdown():
    try:
      trans = tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())
      dx = trans.transform.translation.x
      dy = trans.transform.translation.y
      control_command = Twist()
      if dx ** 2 + dy ** 2 < tol ** 2:
        print("We're here kids.", i)
        pub.publish(control_command)
        return
      control_command.linear.x = K1 * dx
      control_command.angular.z = K2 * dy
      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      pass
    i += 1
    r.sleep()

if __name__ == '__main__':
  rospy.init_node('turtlebot_controller', anonymous=True)
  try:
    moveTo(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
    pass

