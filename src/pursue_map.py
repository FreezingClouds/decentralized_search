""" Class definitions for pursue wrapper """

import numpy as np
import heapq

class Map(object):
    def __init__(self, width, height, grid, resolution, origin):
        self.x_max = width
        self.y_max = height
        self.occupancy = grid
        self.meters_per_cell = resolution
        self.pose_origin = origin
        self.num_swarm_points = np.zeros((self.x_max, self.y_max))

        self.swarm = []
        self.evader_detected = False
        self.evader_location = None
        return

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
        valid = False
        while not valid:
            x, y = np.random.randint(self.x_max), np.random.randint(self.y_max)
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
        # TODO: Debug this method
        assert not self.is_obstacle(location1) and not self.is_obstacle(location2)
        start = (location1[0], location1[1])
        finish = (location2[0], location2[1])

        priority_queue = []
        node_to_prev_node = {}  # maps a node to the previous node that had called dijkstra on it
        visited = set()

        heapq.heappush(priority_queue, (0, start))
        visited.add(start)

        while True:
           if len(priority_queue) == 0:
               break
           distance_to_curr, curr_node = heapq.heappop(priority_queue)
           visited.add(curr_node)
           if curr_node == finish:
               break

           neighbors = self.get_voxel_neighbors(Location(curr_node[0], curr_node[1]))

           for n in neighbors:
               if (n.x, n.y) not in visited:  # if it hasn't been visited, there's a possibility of getting better path
                   nodes_in_pq = (list(map(lambda x: x[1], priority_queue)))
                   if n in nodes_in_pq:  # if it's currently in the pq, update the priority
                       index = nodes_in_pq.index(n)
                       curr_dist = distance_to_curr + self.dist(n, curr_node)
                       prev_dist = priority_queue[index][0]
                       if curr_dist < prev_dist:
                           priority_queue[index] = (curr_dist, n)
                           node_to_prev_node[n] = curr_node
                   else:  # otherwise add it to the pq
                       heapq.heappush(priority_queue, (distance_to_curr + self.dist(n, curr_node), n))
                       node_to_prev_node[n] = curr_node
           prev_node = curr_node

        # Getting the path:
        if finish not in node_to_prev_node.keys():
           return [start]  # Returns only the first node if there is no path

        curr_node = finish
        path = [finish]
        while curr_node != start:
           curr_node = node_to_prev_node[curr_node]
           path.insert(0, curr_node)
        return path

    def dist(self, tup1, tup2):
        return ((tup1[0] - tup2[0]) ** 2 + (tup1[1] - tup2[1]) ** 2) ** (1/2)
