#!/usr/bin/env python
import rospy
import tf2_ros
import sys

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

tol = 0.05

class PursuerNode(object):
  def __init__(self, robot_id):
    self.id = robot_id
    self.request_new_location = rospy.ServiceProxy("/")
    self.pub = rospy.Publisher("/robot0/cmd_vel", Twist, queue_size=10)
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(tfBuffer)
    self.r = rospy.Rate(10)
    self.K1 = 0.3
    self.K2 = 1
    self.curr_target_location = None
    self.run_control_loop()

  def run_control_loop(self, robot_frame):
    while not rospy.is_shutdown():
      x, y = # Get x, y coordinates of robot
      response = self.request_new_location(x=x, y=y, id=robot_id)
      goal = (response.x, response.y)
      self.move_to_location(goal)

  def move_to_location(self, goal):
    # TODO: Convert goal to some kind of dx dy?
    dx = 1000
    dy = 1000
    while (dx ** 2 + dy ** 2) > tol ** 2:
      dx, dy = None, None # Covert here
      self.control_step(dx, dy)
    return 

  def control_step(self, dx, dy):
    control_command.linear.x = K1 * dx
    control_command.angular.z = K2 * dy
    pub.publish(control_command)

  def moveToFrame(robot_frame, target_frame):
    while not rospy.is_shutdown():
      try:
        trans = tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())
        dx = trans.transform.translation.x
        dy = trans.transform.translation.y
        control_command = Twist()
        if dx ** 2 + dy ** 2 < tol ** 2:
          pub.publish(control_command)
          return
        control_command.linear.x = K1 * dx
        control_command.angular.z = K2 * dy
        pub.publish(control_command)
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        pass
      r.sleep()

if __name__ == '__main__':
  # TODO: Get launch file parameters here...
  rospy.init_node('turtlebot_controller', anonymous=True)
  PursuerNode()

