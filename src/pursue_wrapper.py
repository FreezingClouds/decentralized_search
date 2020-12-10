#!/usr/bin/env python
""" Wrapper for multi-agent search and capture. Pseudo-code outlined below"""

import numpy as np
import rospy
import tf2_ros
import time
from decentralized_search.srv import VoxelUpdate, VoxelUpdateResponse
from math import *
from nav_msgs.msg import OccupancyGrid
from pursue_entities import Location, Agent
from pursue_map import Map
from std_msgs.msg import Int8


class Agent_Manager(object):
    def __init__(self, swarm_size=100):
        # Map Initialization
        occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
        array_of_occupancy = np.array(occupancy_grid.data)
        metadata = occupancy_grid.info
        grid = np.vstack(np.split(array_of_occupancy, metadata.height))  # split into metadata.height groups
        self.map = Map(metadata.width, metadata.height, grid, metadata.resolution, metadata.origin)
        self.win_condition = 2.0 / self.map.meters_per_cell  # distance in meters to considered captured

        # Pursuer Initialization (random initial location...subject to change)m
        self.num_pursuers = 3
        self.updated = [False] * self.num_pursuers
        self.map.initialize_swarm(swarm_size)
        self.voxel_detection_distance = int(Agent.detection_radius / float(self.map.meters_per_cell))

        self.claimed_voxels = {i: set() for i in range(self.num_pursuers)}  # for pursuer planning

        for i in range(self.num_pursuers):  # Note: random b/c gets overwritten in 1st call to receive_voxel_update
            self.pursuers = [Agent(i, 0, 0, self.map.meters_per_cell) for i in range(self.num_pursuers)]

        x_evader, y_evader = self.map.get_random_voxel_without_obstacle()
        x_evader, y_evader = 1000, 1000
        self.evader = Agent(3, x_evader, y_evader)

        rospy.Service('/voxel_update', VoxelUpdate, self.receive_voxel_update)  # give new global voxel to travel to

        # For speed up purposes
        self.cue_swarm_update = rospy.Publisher('/update_swarm', Int8, queue_size=10)
        rospy.Subscriber('/update_swarm', Int8, self.receive_cue_to_update_swarm)

        # For speed up purposes
        self.cue_swarm_check = rospy.Publisher('/check_swarm', Int8, queue_size=10)
        rospy.Subscriber('/check_swarm', Int8, self.receive_cue_to_check_swarm)

        # For finish/win indicator to shutdown
        self.finished = rospy.Publisher('/finished', Int8, queue_size=1)

        # For robot pose retrieving purposes
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        voxel_grid = []
        for x in range(self.map.x_max):
            for y in range(self.map.y_max):
                if True:#self.map.is_obstacle(Location(x, y)):
                    voxel_grid.append((x, y))
        # while not rospy.is_shutdown():
        #     rospy.sleep(10)
        #     self.map.visualize_voxels(voxel_grid)

        rospy.spin()
        return

    def receive_voxel_update(self, service_request):
        """ Given a service_request consisting of a location, and respective agent ID,
            return a new voxel location for the agent to travel to."""
        x, y = self.map.location_to_voxel(service_request.x, service_request.y)

        if any([agent.curr_location.distance(self.evader.curr_location) < self.win_condition and self.map.evader_detected[i] for i, agent in enumerate(self.pursuers)]):
            print(' Target CAPTURED! ')
            print([str(agent.curr_location) for agent in self.pursuers])
            print(self.evader.curr_location)
            print([agent.curr_location.distance(self.evader.curr_location) for agent in self.pursuers])
            print(self.win_condition)
            self.finished.publish(Int8(1))
            rospy.sleep(1)
            rospy.signal_shutdown('done')

        if self.is_pursuer(service_request):
            start = time.time()
            agent_id = service_request.id
            self.map.evader_detected[agent_id] = self.in_vision_of(Location(x, y), self.evader.curr_location, visualize=True)
            if self.map.evader_detected[agent_id]:
                self.map.evader_location = self.evader.curr_location
                print('Robot number {} has detected Raylen'.format(agent_id))
            agent = self.pursuers[agent_id]
            agent.curr_location = Location(x, y)
            r = self.voxel_detection_distance

            # Algorithm
            path, updated = agent.get_path(self.map, set.union(*self.claimed_voxels.values()), r)
            if updated:
                claimed = set.union(*[set(self.map.locations_to_tuples(self.map.get_voxel_neighbors(p, r))) for p in path])
                self.claimed_voxels[agent_id] = claimed
            new_location = path.pop(0)  # mutates in place
            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)
            self.updated[agent_id] = True
            self.cue_swarm_check.publish(Int8(1))
            if all(self.updated) or self.map.evader_location:
                self.cue_swarm_update.publish(Int8(1))
            # print('Pursuer planning time: {} seconds'.format(time.time() - start))
        else:
            agent = self.evader
            agent.curr_location = Location(x, y)
            path = agent.get_path_evader(self.map, self.pursuers)
            new_location = path.pop(0)
            coord_x, coord_y = self.map.voxel_to_location(new_location.x, new_location.y)
        coord_x, coord_y = coord_x + self.map.meters_per_cell / 2.0, coord_y + self.map.meters_per_cell / 2.0
        # if service_request.id == 0:
        #     self.map.visualize_voxels([(new_location.x, new_location.y)])
        return VoxelUpdateResponse(coord_x, coord_y)

    def is_pursuer(self, service_request):
        return service_request.id < 3

    def receive_cue_to_update_swarm(self, msg):
        self.update_swarm()
        self.updated = [False] * self.num_pursuers

    def receive_cue_to_check_swarm(self, msg):
        self.check_swarm_detection()

    def update_swarm(self):
        start = time.time()
        pursuer_locations = np.vstack([np.array([p.curr_location.x, p.curr_location.y]) for p in self.pursuers])
        for point in self.map.swarm:
            point.move_one_step(self, self.map, pursuer_locations)
        if time.time() - start > .2:
            print('Warning: updating swarm took {} seconds'.format(time.time() - start))

    def check_swarm_detection(self):
        start = time.time()
        for point in self.map.swarm:
            point.check_detected(self, self.map)
        if time.time() - start > .5:
            print('Warning: checking swarm detection took more than .5 seconds')

    def in_detection_zone(self, location):
        return any([self.in_vision_of(location, agent.curr_location) for agent in self.pursuers])

    def occupied(self, p):
        if p[0] > self.map.x_max or p[0] < 0 or p[1] > self.map.y_max or p[1] < 0:
            return True
        if self.map.is_obstacle(Location(p[0], p[1])):
            return True
        return False

    def in_vision_of(self, location1, location2, view_dist=500, pose=0, fov=2 * pi, visualize=False):
        location1 = Location(*self.map.voxel_to_location(location1.x, location1.y))
        location2 = Location(*self.map.voxel_to_location(location2.x, location2.y))
        ray = self.map.get_vision_ray(location1, location2, view_dist, visualize)
        tar_angle = min(ray[1], 2 * pi - ray[1])
        if abs(pose - tar_angle) <= fov / 2:
            dx = float(location2.x - location1.x)
            dy = float(location2.y - location1.y)
            return ray[0] + self.map.meters_per_cell / 2 >= sqrt(dx ** 2 + dy ** 2)
        print("Out of FOV len={p}, fov: {t} > {f}".format(p=pose, t=tar_angle, f=fov))
        return False


if __name__ == '__main__':
    rospy.init_node('dummy', anonymous=True)
    Agent_Manager()
