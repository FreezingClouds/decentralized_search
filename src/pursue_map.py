""" Class definitions for pursue wrapper """

import heapq
import numpy as np
from math import *

import cv2
import matplotlib.pyplot as plt
import rospy
from decentralized_search.srv import Tolerance
from geometry_msgs.msg import Point
from skimage.measure import block_reduce
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from pursue_entities import Location, SwarmPoint

dilation = 2
shrinkage = 15  # INTEGER. The higher, the more we shrink resolution of Occupancy Grid


class Map(object):
    def __init__(self, width, height, grid, resolution, origin):
        if shrinkage < 1:
            rospy.signal_shutdown('Shrinkage should not be <1. The map is dense enough already you donut.')
            return
        self.occupancy = self.shrink_map(grid, shrinkage)
        plt.imshow(self.occupancy, cmap='hot', origin='lower')
        plt.show()
        self.occupancy = np.swapaxes(self.occupancy, 0, 1)  # need to swap

        self.x_max = self.occupancy.shape[0]
        self.y_max = self.occupancy.shape[1]

        self.meters_per_cell = resolution * shrinkage
        self.pose_origin = origin
        self.num_swarm_points = np.zeros((self.x_max, self.y_max))

        self.swarm = []
        self.evader_detected = [False] * 3

        self.evader_location = None

        self.tolerance_to_set = self.meters_per_cell * .95
        rospy.Service("/tolerance", Tolerance, self.get_tolerance)

        self.neighbor_map = {}  # maps tuple to set of tuples that are non-obstacle neighbors (made for runtime)

        # Used for visualizing vision rays
        self.vis = rospy.Publisher("/vision_rays", Marker, queue_size=10, latch=True)

        return

    def get_tolerance(self, request):
        return self.tolerance_to_set

    def shrink_map(self, grid, shrinkage):
        size = (shrinkage, shrinkage)
        kernel = np.ones((3, 3), np.uint8)
        grid = cv2.dilate(grid.astype(np.uint8), kernel, iterations=dilation)  # Add buffers to walls
        grid = block_reduce(grid, block_size=size, func=np.any)  # Reduce resolution
        return np.ceil(grid).astype(np.float)

    def get_voxel_neighbors(self, location, distance=1):
        neighbors = list(self.tuples_of_box_around_point(location.x, location.y, distance))
        neighbors.remove((location.x, location.y))
        neighbors = [Location(n[0], n[1]) for n in neighbors if 0 <= n[0] < self.x_max and 0 <= n[1] < self.y_max]
        neighbors = [n for n in neighbors if not self.is_obstacle(n)]
        return neighbors

    def tuples_of_box_around_point(self, x, y, distance):
        all_x_deltas, all_y_deltas = range(-distance, distance + 1), range(-distance, distance + 1)
        res = set()
        for curr_x in all_x_deltas:
            for curr_y in all_y_deltas:
                res.add((x + curr_x, y + curr_y))
        return res

    def initialize_swarm(self, swarm_size):
        for i in range(swarm_size):
            x, y = self.get_random_voxel_without_obstacle()
            alpha1, alpha2, alpha3 = np.random.uniform(low=.3, high=.7, size=3)
            self.swarm.append(SwarmPoint(x, y, alpha1, alpha2, alpha3))
            self.num_swarm_points[x, y] += 1
        return

    def get_random_voxel_without_obstacle(self):
        valid, x, y = False, None, None
        while not valid:
            x, y = np.random.randint(self.x_max), np.random.randint(self.y_max)
            valid = not self.is_obstacle(Location(x, y))
        return x, y

    def distance(self, agent1, agent2):
        return agent1.distance(agent2.curr_location)

    def voxel_to_location(self, x, y):
        location_x = self.pose_origin.position.x + self.meters_per_cell * x
        location_y = self.pose_origin.position.y + self.meters_per_cell * y
        return location_x, location_y

    def location_to_voxel(self, x, y):
        voxel_x = round((x - self.pose_origin.position.x) / self.meters_per_cell)
        voxel_y = round((y - self.pose_origin.position.y) / self.meters_per_cell)
        return int(voxel_x), int(voxel_y)

    def is_obstacle(self, location):
        return self.occupancy[location.x, location.y] > .8

    def get_path_opt(self, location1, location2, unallowed=set()):
        start = (location1.x, location1.y)
        finish = (location2.x, location2.y)

        priority_queue = []
        node_to_prev_node = {}  # maps a node to the previous node that had called dijkstra on it
        visited = set()

        heapq.heappush(priority_queue, (0, start))
        visited.add(start)

        while True:
            if len(priority_queue) == 0:
                break
            distance_to_curr, curr_location = heapq.heappop(priority_queue)
            visited.add(curr_location)
            if curr_location == finish:
                break

            if curr_location in self.neighbor_map:
                neighbors = self.neighbor_map[curr_location]
            else:
                neighbors = self.get_voxel_neighbors(Location(curr_location[0], curr_location[1]), 1)
                neighbors = self.locations_to_tuples(neighbors)
                neighbors = [n for n in neighbors if n not in unallowed]
                self.neighbor_map[curr_location] = neighbors

            for n in neighbors:
                if n not in visited:  # if it hasn't been visited, there's a possibility of getting better path
                    nodes_in_pq = (list(map(lambda x: x[1], priority_queue)))
                    if n in nodes_in_pq:  # if it's currently in the pq, update the priority
                        index = nodes_in_pq.index(n)
                        curr_neighbor_dist = distance_to_curr + self.get_dist_between_tuples(curr_location, n)
                        prev_dist = priority_queue[index][0] - self.get_dist_between_tuples(n, finish)  # Subtract out heuristic
                        if curr_neighbor_dist < prev_dist:
                            priority_queue[index] = (curr_neighbor_dist + self.get_dist_between_tuples(n, finish), n)  # Add heuristic
                            node_to_prev_node[n] = curr_location
                    else:  # otherwise add it to the pq
                        heapq.heappush(priority_queue, (distance_to_curr + self.get_dist_between_tuples(curr_location, n) + self.get_dist_between_tuples(n, finish), n))
                        node_to_prev_node[n] = curr_location

        # Getting the path:
        if finish not in node_to_prev_node.keys():
            return self.tuples_to_locations([start])  # Returns only the first node if there is no path

        curr_node = finish
        path = [finish]
        while curr_node != start:
            curr_node = node_to_prev_node[curr_node]
            path.insert(0, curr_node)
        return self.tuples_to_locations(path)[1:]

    def get_path(self, location1, location2, unallowed=set()):
        # NOTE: All locations in this method are tuples representing voxels for efficiency
        assert not self.is_obstacle(location1)
        assert not self.is_obstacle(location2)
        return self.get_path_opt(location1, location2, unallowed=set())

    def tuples_to_locations(self, list_of_tuples):
        return [Location(t[0], t[1]) for t in list_of_tuples]

    def locations_to_tuples(self, list_of_locations):
        return [(l.x, l.y) for l in list_of_locations]

    def get_dist_between_tuples(self, tup1, tup2):
        return np.sqrt((tup1[0] - tup2[0]) ** 2 + (tup1[1] - tup2[1]) ** 2)

    def nearest_non_obstacles(self, location):
        if location.x < 1 or location.x >= self.x_max or location.y < 1 or location.y >= self.y_max:
          print('Out of bounds!')
        voxHeap = []
        heapq.heappush(voxHeap, (0, (location.x, location.y)))
        alreadyVisited = set()
        while True:
          currLocation = heapq.heappop(voxHeap)
          currLocation = currLocation[1]
          if not self.is_obstacle(Location(currLocation[0], currLocation[1])):
            print('Done!')
            return Location(currLocation[0], currLocation[1])
          neighbors = list(self.tuples_of_box_around_point(currLocation[0], currLocation[1], 1))
          neighbors.remove(currLocation)
          alreadyVisited.add(currLocation)
          if (location.x, location.y) in neighbors:
            neighbors.remove((location.x, location.y))
          nodes_in_pq = (list(map(lambda x: x[1], voxHeap)))
          neighbors = [Location(n[0], n[1]) for n in neighbors if (0 <= n[0] < self.x_max) and (0 <= n[1] < self.y_max) and (n not in alreadyVisited) and (n not in nodes_in_pq)]
          for loc in neighbors:
            heapq.heappush(voxHeap, (loc.distance(location), (loc.x, loc.y)))
  
    def get_vision_ray(self, location1, location2, max_range=200, visualize=False):
        dx = float(location2.x - location1.x)
        dy = float(location2.y - location1.y)
        theta = pi
        if dx == 0:
            theta = pi / 2 if dy > 0 else -pi / 2
        elif dx > 0:
            theta = atan(dy / dx)
        else:
            theta += atan(dy / dx)
        dist = 1
        vis = list()
        while dist <= max_range / self.meters_per_cell:
            xMap = int(location1.x / self.meters_per_cell + cos(theta) * dist)
            yMap = int(location1.y / self.meters_per_cell + sin(theta) * dist)
            if visualize: vis.append((xMap, yMap))
            if xMap < 0 or xMap > self.x_max or yMap < 0 or yMap > self.y_max:
                if visualize: print("VISION: Out of bounds at voxel {x}, {y}".format(x=xMap, y=yMap))
                break
            if self.is_obstacle(Location(xMap, yMap)):
                if visualize: print("VISION: Hit wall at voxel {x}, {y}".format(x=xMap, y=yMap))
                break
            dist += 1

        if visualize: self.visualize_voxels(vis, True)
        # vip variable
        rayLen = min(dist * self.meters_per_cell, max_range)
        return rayLen, theta

    def visualize_voxels(self, voxels, is_ray=False):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map_static"
        m.ns = "vision"
        m.id = 0
        m.type = Marker.POINTS
        m.action = Marker.ADD
        m.scale.x = self.meters_per_cell
        m.scale.y = self.meters_per_cell
        m.scale.z = 0.01
        for i, (x, y) in enumerate(voxels):
            voxel_loc = Location(x, y)
            x = x * self.meters_per_cell + self.meters_per_cell / 2
            y = y * self.meters_per_cell + self.meters_per_cell / 2
            m.points.append(Point(x, y, 0))
            if (is_ray and i == len(voxels) - 1) or self.is_obstacle(voxel_loc):
                m.colors.append(ColorRGBA(1, 0.3, 0.2, 0.4))
            else:
                m.colors.append(ColorRGBA(0, 0.5, 1, 0.4))
        self.vis.publish(m)
        print("VISION: Published voxels.")
