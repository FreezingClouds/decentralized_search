#!/usr/bin/env python
""" Wrapper for multi-agent search and capture. Pseudo-code outlined below"""

from pursue_entities import Location, Agent
from pursue_map import Map
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from decentralized_search.srv import VoxelUpdate, VoxelUpdateResponse
from decentralized_search.msg import EvaderLocation


shrinkage = 50  # INTEGER. The higher, the more we shrink resolution of Occupancy Grid

class Agent_Manager(object):
    def __init__(self, swarm_size=100):
        # Map Initialization
        occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
        array_of_occupancy = np.array(occupancy_grid.data)
        metadata = occupancy_grid.info
        grid = np.vstack(np.split(array_of_occupancy, metadata.height))  # split into metadata.height groups
        self.map = Map(metadata.width, metadata.height, grid, metadata.resolution, metadata.origin, shrinkage=shrinkage)

        # Pursuer Initialization (random initial location...subject to change)
        self.num_pursuers = 3
        self.updated = [False] * self.num_pursuers
        self.map.initialize_swarm(swarm_size)
        self.voxel_detection_distance = int(Agent.detection_radius / float(self.map.meters_per_cell))

        rospy.Service('/voxel_update', VoxelUpdate, self.receive_voxel_update)  # give new global voxel to travel to
        rospy.Subscriber('/evader_location', EvaderLocation, self.receive_evader_location, queue_size=1)  # if available, get location of evader
        self.claimed_voxels = {i: set() for i in range(self.num_pursuers)}  # for pursuer planning

        for i in range(self.num_pursuers):  # Note: random b/c gets overwritten in 1st call to receive_voxel_update
            x, y = self.map.get_random_voxel_without_obstacle()
            self.pursuers = [Agent(i, x, y, self.map.meters_per_cell) for i in range(self.num_pursuers)]

        x_evader, y_evader = self.map.get_random_voxel_without_obstacle()
        self.evader = Agent(3, x_evader, y_evader)

        rospy.spin()
        return

    def receive_evader_location(self, msg):
        # Callback for receiving evader location.
        if msg.found:
            self.map.evader_location = Location(msg.x, msg.y)
        else:
            self.map.evader_location = None

    def receive_voxel_update(self, service_request):
        """ Given a service_request consisting of a location, and respective agent ID,
            return a new voxel location for the agent to travel to."""
        x, y = self.map.location_to_voxel(service_request.x, service_request.y)
        
        if self.is_pursuer(service_request):
            agent_id = service_request.id
            agent = self.pursuers[agent_id]
            agent.curr_location = Location(x, y)
            r = self.voxel_detection_distance
            path = agent.get_path(self.map, set.union(*self.claimed_voxels.values()), self.map.evader_location, r)
            claimed = set.union(*[set(self.map.locations_to_tuples(self.map.get_voxel_neighbors(p, r))) for p in path])
            self.claimed_voxels[agent_id] = claimed
            new_location = path.pop(0)  # Note: checked. The bug is not here. new_location is never an obstacle

            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)

            self.updated[agent_id] = True
            self.check_swarm_detection()
            if all(self.updated) or self.map.evader_location:
                self.update_swarm()
                self.updated = [False] * self.num_pursuers
            return VoxelUpdateResponse(coord_x, coord_y)
        agent = self.evader
        agent.curr_location = Location(x, y)
        path = agent.get_path_evader(self.map, self.pursuers)
        new_location = path.pop(0)
        coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)
        return VoxelUpdateResponse(coord_x, coord_y)

    def is_pursuer(self, service_request):
        return service_request.id < 3

    def update_swarm(self):
        for point in self.map.swarm:
            point.move_one_step(self, self.map)

    def check_swarm_detection(self):
        for point in self.map.swarm:
            point.check_detected(self, self.map)

    def occupied(self, p):
        if p[0] > self.map.x_max or p[0] < 0 or p[1] > self.map.y_max or p[1] < 0:
            return True
        if self.map.is_obstacle(Location(p[0], p[1])):
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    Agent_Manager()
