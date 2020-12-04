""" Wrapper for multi-agent search and capture. Pseudo-code outlined below"""

from pursue_entities import *
from pursue_map import Map
import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Vector3

class Pursuit(object):
    def __init__(self, swarm_size=100, time_steps = 1000):
        # Map Initialization
        occupancy_grid = rospy.wait_for_message('/map')
        array_of_occupancy = occupancy_grid.data
        metadata = occupancy_grid.info
        grid = np.vstack(np.split(array_of_occupancy, metadata.height))  # split into metadata.height groups

        self.map = Map(metadata.width, metadata.height, grid, metadata.resolution, metadata.origin)
        self.map.initialize_swarm(swarm_size)

        self.num_pursuers = 3
        for i in range(self.num_pursuers):
            x, y = self.map.get_random_voxel_without_obstacle()
            self.pursuers = [Agent(i, x, y) for i in range(self.num_pursuers)]
        self.updated = [False] * self.num_pursuers

        s = rospy.Service('voxel_update', _.srv.VoxelUpdate, self.receive_voxel_update)
        rospy.Subscriber('evader_location', Vector3, self.receive_evader_location, queue_size=1)
        self.claimed_paths = {i: [] for i in range(self.num_pursuers)}

        rospy.spin()
        return

    def receive_evader_location(self, msg):
        if msg.x:
            self.map.evader_location = Location(msg.x, msg.y)
        else:
            self.map.evader_location = None

    def receive_voxel_update(self, service_request):
        """ Given a service_request consisting of a location, and respective agent ID,
            return a new voxel location for the agent to travel to."""
            location = service_request.location
            id = service_request.id
            x, y = location[0], location[1]
            x, y = self.map.location_to_voxel(x, y)

            agent = self.pursuers[id]
            agent.curr_location = Location(x, y)
            path = agent.get_path(self.map, self.claimed_paths.values(), self.map.evader_location)

            self.claimed_paths[id] = path
            new_location = path.pop(0)
            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)

            self.updated[id] = True
            if all(self.updated) or not self.map.evader_location:
                self.update_swarm()
                self.updated = [False] * self.num_pursuers
        return _.srv.VoxelResponse(coord_x, coord_y)

    def update_swarm(self):
        for point in self.map.swarm:
            point.move_one_step(self)
        return

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    Pursuit()
