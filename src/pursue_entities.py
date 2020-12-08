import numpy as np
from collections import OrderedDict
import time
max_intersection = .5
from scipy.optimize import minimize

class Location(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance(self, location):
        return np.sqrt(np.square(location.x - self.x) + np.square(location.y - self.y))

    def equal_to(self, location):
        return location.x == self.x and location.y == self.y

class Agent(object):
    detection_radius = 1  # defined in meters
    update_every = 5  # defined in meters

    def __init__(self, id, x, y, meters_per_cell=1):
        self.id = id
        self.curr_location = Location(x, y)
        self.curr_path = []
        self.update_every_k_steps = Agent.update_every / meters_per_cell
        self.counter = 0
        return

    def get_path(self, map, claimed_voxels, evader_location=None, max_intersection=.5, r=1):
        self.counter += 1
        map.detected_evader(evader_location)
        start = time.time()
        if self.counter == self.update_every_k_steps or len(self.curr_path) == 0:
            self.counter, valid_path = 0, False
            locations = [s.curr_location for s in map.swarm if (s.curr_location.x, s.curr_location.y) not in claimed_voxels]
            boundaries = [((max(loc.x - r, 0), min(loc.x + r, map.x_max)), (max(loc.y - r, 0), min(loc.y + r, map.y_max))) for loc in locations]
            densities = [np.sum(map.num_swarm_points[x_bound[0]: x_bound[1] + 1, y_bound[0]: y_bound[1] + 1]) for x_bound, y_bound in boundaries]
            locations_to_densities = dict(zip(locations, densities))
            locations_to_densities = iter(sorted(locations_to_densities.items(), key=lambda item: item[1], reverse=True))
            path = [self.curr_location]
            while not valid_path:
                try:
                    new_location, density = next(locations_to_densities)
                except StopIteration as e:
                    self.curr_path = path  # Edge case
                    break
                if self.curr_location.equal_to(new_location):
                    continue
                path = map.get_path(self.curr_location, new_location)
                tuple_path = map.locations_to_tuples(path)
                too_similar_to_prev_path = len(set(tuple_path).intersection(claimed_voxels)) / float(len(tuple_path)) > max_intersection
                if not too_similar_to_prev_path:
                    valid_path = True
                    self.curr_path = path
        if start - time.time() > 2:
            print('WARNING: Pursuer planning time exceeding 2 seconds...')
        return self.curr_path

    def get_path_evader(self, map, pursuers):
        self.counter += 1
        if self.counter == self.update_every_k_steps or len(self.curr_path) == 0:
            bnds = ((1, map.x_max-1), (1, map.y_max-1))
            x0 = (1,1)
            res = minimize(self.distanceSum, x0, args = (map, pursuers), bounds = bnds, method = 'TNC')
            xCoord = res.x[0]
            yCoord = res.x[1]
            point = (int(xCoord), int(yCoord))
            destination = Location(point[0], point[1])
            path = map.get_path(self.curr_location, destination)
            if len(path) == 0:
                path = [self.curr_location]
            self.curr_path = path
        return self.curr_path

    def distanceSum(self, coords, map, pursuers):
        #eLoc = Location(p[0], p[1])
        x = int(coords[0])
        y = int(coords[1])
        eLoc = Location(x, y)
        p1 = pursuers[0].curr_location
        p2 = pursuers[1].curr_location
        p3 = pursuers[2].curr_location
        #dist1 = eLoc.distance(p1)
        #dist2 = eLoc.distance(p2)
        #dist3 = eLoc.distance(p3)
        #return dist1 + dist2 + dist3

        d1 = map.get_path(eLoc, p1)
        d2 = map.get_path(eLoc, p2)
        d3 = map.get_path(eLoc, p3)

        d1 = len(d1)
        d2 = len(d2)
        d3 = len(d3)
        distArray = np.array([d1, d2, d3])
        sortedArray = np.sort(distArray)
        #print(sortedArray)

        return -(0.01*sortedArray[0] + 0.1*sortedArray[1] + 10*sortedArray[2])

class SwarmPoint(Agent):
    def __init__(self, x, y, alpha1, alpha2, alpha3):
        super(SwarmPoint, self).__init__(-1, x, y, meters_per_cell=1)
        self.alphas = (alpha1, alpha2, alpha3)
        return

    def move_one_step(self, wrapper, map):
        map.num_swarm_points[self.curr_location.x, self.curr_location.y] -= 1
        if map.evader_detected:
            self.curr_location = map.evader_location
        else:  # evader not detected and swarmpoint not in detection zone
            neighbors = map.get_voxel_neighbors(self.curr_location)
            neighbors.append(self.curr_location)
            neighbor_values = [self.compute_weighted_sum(neighbor_location, wrapper) for neighbor_location in neighbors]
            best_neighbor = np.argmax(neighbor_values)
            choice = neighbors[best_neighbor]
            self.curr_location = choice

        map.num_swarm_points[self.curr_location.x, self.curr_location.y] += 1
        return

    def check_detected(self, wrapper, map):
        if map.in_detection_zone(self.curr_location):
            swarm_locations = [s.curr_location for s in map.swarm]
            swarm_locations.remove(self.curr_location)
            self.curr_location = np.random.choice(swarm_locations)

    def compute_weighted_sum(self, location, wrapper):
        return sum([p.curr_location.distance(location) * self.alphas[i] for i, p in enumerate(wrapper.pursuers)])
