#!/usr/bin/env python
""" Wrapper for multi-agent search and capture. Pseudo-code outlined below"""

from pursue_entities import Location, Agent, SwarmPoint
from pursue_map import Map
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3

class Pursuit(object):
    def __init__(self, swarm_size=100, time_steps = 1000):
        # Map Initialization
        occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
        array_of_occupancy = np.array(occupancy_grid.data)
        metadata = occupancy_grid.info
        grid = np.vstack(np.split(array_of_occupancy, metadata.height))  # split into metadata.height groups

        self.map = Map(metadata.width, metadata.height, grid, metadata.resolution, metadata.origin)
        self.map.initialize_swarm(swarm_size)

        # Pursuer Initialization (random initial location...subject to change)
        self.num_pursuers = 3
        for i in range(self.num_pursuers):
            x, y = self.map.get_random_voxel_without_obstacle()
            self.pursuers = [Agent(i, x, y) for i in range(self.num_pursuers)]
        self.updated = [False] * self.num_pursuers

        xe, ye = self.map.get_random_voxel_without_obstacle()
        self.evader = Agent(3, xe, ye)

        # voxel_update: service to give pursuers new global map location (not a voxel) to travel to
        # evader_location: subscribe to message that publishes location of evader if available
        s = rospy.Service('/voxel_update', decentralized_search.srv.VoxelUpdate, self.receive_voxel_update)
        rospy.Subscriber('/evader_location', decentralized_search.msg.EvaderLocation, self.receive_evader_location, queue_size=1)
        self.claimed_paths = {i: [] for i in range(self.num_pursuers)}

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
        if service_request.id < 3
            agent_id = service_request.id
            x, y = self.map.location_to_voxel(service_request.x, service_request.y)

            agent = self.pursuers[agent_id]
            agent.curr_location = Location(x, y)
            path = agent.get_path(self.map, self.claimed_paths.values(), self.map.evader_location)

            self.claimed_paths[agent_id] = path
            new_location = path.pop(0)
            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)

            self.updated[agent_id] = True
            if all(self.updated) or not self.map.evader_location:
                self.update_swarm()
                self.updated = [False] * self.num_pursuers
            return decentralized_search.srv.VoxelUpdateResponse(coord_x, coord_y)

        xe, ye = self.map.location_to_voxel(service_request.x, service_request.y)
        targetPoint = np.array([xe, ye])
        for i in range(-1, 2):
            for k in range(-1, 2):
                currPoint = np.array([xe + i, ye + k])
                if not occupied(currPoint):
                    if distanceSum(currPoint) > distanceSum(targetPoint):
                        targetPoint = currPoint
        return decentralized_search.srv.VoxelUpdateResponse(targetPoint[0], targetPoint[1])

    def update_swarm(self):
        for point in self.map.swarm:
            point.move_one_step(self)
        return

    def distanceSum(p):
        eLoc = Location(p[0], p[1])
        p1 = self.pursuers[0].curr_location
        p2 = self.pursuers[1].curr_location
        p3 = self.pursuers[2].curr_location
        dist1 = eLoc.distance(p1)
        dist2 = eLoc.distance(p2)
        dist3 = eLoc.distance(p3)
        return dist1 + dist2 + dist3

    def occupied(p):
        if p[0] > self.map.x_max || p[0] < 0 || p[1] > self.map.y_max || p[1] < 0:
            return True
        if self.map.is_obstacle(Location(p[0], p[1])):
            return True
        return False

if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    Pursuit()
