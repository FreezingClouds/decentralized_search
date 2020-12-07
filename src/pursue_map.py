""" Class definitions for pursue wrapper """

import numpy as np
import heapq
from pursue_entities import Location, SwarmPoint
import rospy
from skimage.measure import block_reduce

class Map(object):
    def __init__(self, width, height, grid, resolution, origin, shrinkage=1):
        if shrinkage < 1:
            rospy.signal_shutdown('Shrinkage should not be <1. The map is dense enough already you donut.')
            return
        if width % shrinkage != 0 or height % shrinkage != 0:
            rospy.signal_shutdown('Shrinkage not divisible by width or height you donut')

        self.occupancy = self.shrink_map(grid, shrinkage)
        self.x_max = self.occupancy.shape[0]
        self.y_max = self.occupancy.shape[1]
        self.meters_per_cell = resolution * shrinkage
        self.pose_origin = origin
        self.num_swarm_points = np.zeros((self.x_max, self.y_max))

        self.swarm = []
        self.evader_detected = False
        self.evader_location = None
        return

    def shrink_map(self, grid, shrinkage):
        # Convolve with specific square kernel of all ones
        size = (shrinkage, shrinkage)
        return block_reduce(grid, block_size=size, func=np.any)

    def get_voxel_neighbors(self, location, distance=1):
        all_x_deltas, all_y_deltas = range(-distance, distance+1), range(-distance, distance+1)
        neighbors = []
        for x in all_x_deltas:
            for y in all_y_deltas:
                neighbors.append((location.x + x, location.y + y))
        neighbors.remove((location.x, location.y))
        neighbors = [Location(n[0], n[1]) for n in neighbors if 0 <= n[0] < self.x_max and 0 <= n[1] < self.y_max]
        neighbors = [n for n in neighbors if not self.is_obstacle(n)]
        return neighbors

    def initialize_swarm(self, swarm_size):
        for i in range(swarm_size):
            x, y = self.get_random_voxel_without_obstacle()
            alpha1, alpha2, alpha3 = np.random.uniform(size=3)
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

    def detected_evader(self, location):
        self.evader_detected = False if not location else True
        self.evader_location = location

    def voxel_to_location(self, x, y):
        location_x = self.pose_origin.position.x + self.meters_per_cell * x
        location_y = self.pose_origin.position.y + self.meters_per_cell * y
        return location_x, location_y

    def location_to_voxel(self, x, y):
        voxel_x = int((x - self.pose_origin.position.x) / self.meters_per_cell)
        voxel_y = int((y - self.pose_origin.position.y) / self.meters_per_cell)
        return voxel_x, voxel_y

    def is_obstacle(self, location):
        return self.occupancy[location.x, location.y] > .8

    def in_detection_zone(self, location):
        # TODO: Plug into Raylen's vision code
        return

    def get_path(self, location1, location2):
        # NOTE: All locations in this method are tuples representing voxels for efficiency
        assert not self.is_obstacle(location1) and not self.is_obstacle(location2)
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

           neighbors = self.get_voxel_neighbors(Location(curr_location[0], curr_location[1]))
           neighbors = [(n.x, n.y) for n in neighbors]

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
        # print(finish in node_to_prev_node.keys())

        # Getting the path:
        if finish not in node_to_prev_node.keys():
            return self.tuples_to_locations([start])  # Returns only the first node if there is no path

        curr_node = finish
        path = [finish]
        while curr_node != start:
           curr_node = node_to_prev_node[curr_node]
           path.insert(0, curr_node)
        return self.tuples_to_locations(path)



    def tuples_to_locations(self, list_of_tuples):
        return [Location(t[0], t[1]) for t in list_of_tuples]

    def get_dist_between_tuples(self, tup1, tup2):
        return np.sqrt((tup1[0] - tup2[0]) ** 2 + (tup1[1] - tup2[1]) ** 2)
