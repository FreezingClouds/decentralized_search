#!/usr/bin/env python
""" Wrapper for multi-agent search and capture. Pseudo-code outlined below"""

from pursue_entities import Location, Agent, R
from pursue_map import Map
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from decentralized_search.srv import VoxelUpdate, VoxelUpdateResponse
from decentralized_search.msg import EvaderLocation

shrinkage = 25  # INTEGER. The higher, the more we shrink resolution of Occupancy Grid


class Pursuit(object):
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

        # voxel_update: service to give pursuers new global map location (not a voxel) to travel to
        # evader_location: subscribe to message that publishes location of evader if available
        rospy.Service('/voxel_update', VoxelUpdate, self.receive_voxel_update)
        rospy.Subscriber('/evader_location', EvaderLocation, self.receive_evader_location, queue_size=1)
        self.claimed_voxels = {i: set() for i in range(self.num_pursuers)}

        # Note: we are able to set random location because this gets overwritten anyway
        for i in range(self.num_pursuers):
            x, y = self.map.get_random_voxel_without_obstacle()
            self.pursuers = [Agent(i, x, y) for i in range(self.num_pursuers)]

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
        if self.is_pursuer(service_request):
            agent_id = service_request.id
            x, y = self.map.location_to_voxel(service_request.x, service_request.y)

            agent = self.pursuers[agent_id]
            agent.curr_location = Location(x, y)
            path = agent.get_path(self.map, set.union(*self.claimed_voxels.values()), self.map.evader_location)
            if agent_id == 0:
                print(len(path))

            claimed = set.union(*[set(self.map.locations_to_tuples(self.map.get_voxel_neighbors(p, R))) for p in path])
            self.claimed_voxels[agent_id] = claimed
            new_location = path.pop(0)
            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)

            self.updated[agent_id] = True
            self.check_swarm_detection()
            if all(self.updated) or self.map.evader_location:
                self.update_swarm()  # NOTE: Is this if statement right? If in detection_zone, should update it NOW
                self.updated = [False] * self.num_pursuers
            return VoxelUpdateResponse(coord_x, coord_y)

        x_evader, y_evader = self.map.location_to_voxel(service_request.x, service_request.y)
        targetPoint = np.array([x_evader, y_evader])
        for i in range(-1, 2):
            for k in range(-1, 2):
                currPoint = np.array([x_evader + i, y_evader + k])
                if not self.occupied(currPoint):
                    if self.distanceSum(currPoint) > self.distanceSum(targetPoint):
                        targetPoint = currPoint
        return VoxelUpdateResponse(targetPoint[0], targetPoint[1])

    def is_pursuer(self, service_request):
        return service_request.id < 3

    def update_swarm(self):
        for point in self.map.swarm:
            point.move_one_step(self, self.map)

    def check_swarm_detection(self):
        for point in self.map.swarm:
            point.check_detected(self, self.map)

    def distanceSum(self, p):
        eLoc = Location(p[0], p[1])
        p1 = self.pursuers[0].curr_location
        p2 = self.pursuers[1].curr_location
        p3 = self.pursuers[2].curr_location
        dist1 = eLoc.distance(p1)
        dist2 = eLoc.distance(p2)
        dist3 = eLoc.distance(p3)
        return dist1 + dist2 + dist3

    def occupied(self, p):
        if p[0] > self.map.x_max or p[0] < 0 or p[1] > self.map.y_max or p[1] < 0:
            return True
        if self.map.is_obstacle(Location(p[0], p[1])):
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    Pursuit()
