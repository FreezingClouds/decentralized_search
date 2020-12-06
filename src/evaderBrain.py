# what to do after receiving position of pursuers (voxel coords)? p1 = [x1, y1], p2 = [x2, y2], p3 = [x3, y3], pe = [xe, ye]
# Generate a set of occupied voxels: walls
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from pursue_map import Map
from pursue_entities import Location, Agent

class Evader(object):
	def __init__(self):
		occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
		array_of_occupancy = np.array(occupancy_grid.data)
		metadata = occupancy_grid.info
		self.map = Map(metadata.width, metadata.height, metadata.resolution, metadata.origin)

		# Initialize evader
		x, y = self.map.get_random_voxel_without_obstacle()
		self.evader = Agent(3, x, y)

		s = rospy.Service('/target_location', decentralized_search.srv.VoxelUpdate, self.nextVoxel)
		rospy.Subscriber('/pursuer_location', decentralized_search.msg.PursuerLocations, self.pursuerLocs)

		rospy.spin()

	def pursuerLocs(self, msg):
		self.p1 = Agent(0, msg.p1.x, msg.p1.y)
		self.p2 = Agent(1, msg.p2.x, msg.p2.y)
		self.p3 = Agent(2, msg.p3.x, msg.p3.y)

	def nextVoxel(self, service_request):
		xe, ye = self.map.location_to_voxel(service_request.x, service_request.y)
		targetPoint = np.array([xe, ye])
		for i in range(-1, 2):
			for k in range(-1, 2):
				currPoint = np.array([xe + i, ye + k])
				if not occupied(currPoint):
					if distanceSum(currPoint) > distanceSum(targetPoint):
						targetPoint = currPoint
		return decentralized_search.srv.VoxelUpdateResponse(targetPoint[0], targetPoint[1])

	def distanceSum(p):
		ag = Agent(4, p[0], p[1])
		dist1 = self.map.distance(ag, self.p1)
		dist2 = self.map.distance(ag, self.p2)
		dist3 = self.map.distance(ag, self.p3)
		return dist1 + dist2 + dist3

	def occupied(p):
		if p[0] > self.map.x_max || p[0] < 0 || p[1] > self.map.y_max || p[1] < 0:
			return True
		if self.map.is_obstacle(Location(p[0], p[1])):
			return True
		return False

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('evaderController', anonymous=True)
  Evader()