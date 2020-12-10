import numpy as np
from collections import OrderedDict
import time
from scipy.optimize import differential_evolution

max_intersection = .2

class Location(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance(self, location):
        return np.sqrt(np.square(location.x - self.x) + np.square(location.y - self.y))

    def equal_to(self, location):
        return location.x == self.x and location.y == self.y

    def __str__(self):
        return "({x}, {y})".format(x=self.x, y=self.y)

class Agent(object):
    detection_radius = 10  # defined in meters
    update_every = 2  # defined in meters

    def __init__(self, id, x, y, meters_per_cell=1):
        self.id = id
        self.curr_location = Location(x, y)
        self.curr_path = []
        self.update_every_k_steps = Agent.update_every / meters_per_cell
        self.counter = 0
        return

    def get_path(self, map, claimed_voxels, r=1):
        if self.counter == self.update_every_k_steps or len(self.curr_path) == 0 \
                or (any(map.evader_detected) and not self.curr_path[-1].equal_to(map.evader_location)):
            self.counter, valid_path = 0, False
            if any(map.evader_detected):
                locations_to_densities = iter({map.evader_location: 20}.items())
                num_options = 1
            else:
                locations = [s.curr_location for s in map.swarm if (s.curr_location.x, s.curr_location.y) not in claimed_voxels]
                if len(locations) == 0:
                    locations = []
                    covered = set()
                    prev = [s.curr_location for s in map.swarm if (s.curr_location.x, s.curr_location.y)]
                    for loc in prev:
                        if (loc.x, loc.y) not in covered:
                            locations.append(loc)
                            covered = covered.union(map.tuples_of_box_around_point(loc.x, loc.y, r))

                boundaries = [((max(loc.x - r, 0), min(loc.x + r, map.x_max)), (max(loc.y - r, 0), min(loc.y + r, map.y_max))) for loc in locations]
                densities = [np.sum(map.num_swarm_points[x_bound[0]: x_bound[1] + 1, y_bound[0]: y_bound[1] + 1]) for x_bound, y_bound in boundaries]
                locations_to_densities = dict(zip(locations, densities))
                locations_to_densities = OrderedDict(sorted(locations_to_densities.items(), key=lambda item: item[1], reverse=True))

                # RUNTIME STUFF: Filtering for algorithm runtime boost
                filtered = OrderedDict()
                covered = set()
                for loc, density in locations_to_densities.items():
                    if (loc.x, loc.y) not in covered:
                        filtered[loc] = density
                        covered = covered.union(map.tuples_of_box_around_point(loc.x, loc.y, r))

                locations_to_densities = iter(filtered.items())
                num_options = len(filtered)
            least_resistance_path, resistance = [self.curr_location], np.inf

            while not valid_path:
                try:
                    new_location, density = next(locations_to_densities)
                except StopIteration as e:
                    self.curr_path = least_resistance_path  # Edge case
                    break
                if any(map.evader_detected) or num_options < 3:
                    path = map.get_path(self.curr_location, new_location, claimed_voxels)
                    path = map.get_path(self.curr_location, new_location) if len(path) <= 1 else path
                else:
                    path = map.get_path(self.curr_location, new_location)
                if len(path) <= 1:
                    continue
                tuple_path = map.locations_to_tuples(path)
                resist = len(set(tuple_path).intersection(claimed_voxels)) / float(len(tuple_path))
                print(resist)
                valid_path = resistance <= max_intersection
                self.curr_path = path
                least_resistance_path = path if resist < resistance else least_resistance_path
                resistance = min(resist, resistance)

        updated = self.counter == 0
        self.counter += 1
        return self.curr_path, updated

    def get_path_evader(self, map, pursuers):
        self.counter += 1
        if self.counter == self.update_every_k_steps or len(self.curr_path) == 0:
            bnds = [(1, map.x_max - 2), (1, map.y_max - 2)]
            # print(map.x_max, map.y_max)
            res = differential_evolution(self.distanceSum, bnds, args=(map, pursuers), maxiter=1000)
            xCoord = res.x[0]
            yCoord = res.x[1]

            point = (int(np.round(xCoord)), int(np.round(yCoord)))
            destination = Location(point[0], point[1])
            if map.is_obstacle(destination):
                destination = map.nearest_non_obstacles(destination)
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
        d1 = eLoc.distance(p1)
        d2 = eLoc.distance(p2)
        d3 = eLoc.distance(p3)
        # d1 = map.get_path_opt(eLoc, p1)
        # d2 = map.get_path_opt(eLoc, p2)
        # d3 = map.get_path_opt(eLoc, p3)
        #
        # d1 = len(d1)
        # d2 = len(d2)
        # d3 = len(d3)
        distArray = np.array([d1, d2, d3])
        sortedArray = np.sort(distArray)
        #print(sortedArray)

        return np.exp(-sortedArray[0]) + np.exp(-sortedArray[1]) + np.exp(-sortedArray[2])

class SwarmPoint(Agent):
    def __init__(self, x, y, alpha1, alpha2, alpha3):
        super(SwarmPoint, self).__init__(-1, x, y, meters_per_cell=1)
        self.alphas = np.expand_dims(np.array([alpha1, alpha2, alpha3]), axis=1)
        self.was_evader_location = False
        self.prev_evader_location = None
        self.detected_since_evader = False
        return

    def move_one_step(self, wrapper, map, pursuer_location_array):
        old_loc = self.curr_location
        if any(map.evader_detected):
            self.curr_location = map.evader_location
            self.was_evader_location = True
            self.prev_evader_location = (map.evader_location.x, map.evader_location.y)
            self.detected_since_evader = False
        else:
            neighbors = map.get_voxel_neighbors(self.curr_location, distance=1)
            neighbors.append(self.curr_location)
            num_neighbors = len(neighbors)
            neighbor_array = np.expand_dims(np.array([[l.x, l.y] for l in neighbors]), axis=0)  # 1 x neighbors x 2
            neighbor_array = np.repeat(neighbor_array, 3, axis=0)  # 3 x neighbors x 2
            pursuer_location_array = np.repeat(np.expand_dims(pursuer_location_array, axis=1), num_neighbors, axis=1)  # 3 x neighbors x 2
            square_dist = np.sqrt(np.sum(np.square(neighbor_array - pursuer_location_array), axis=2))  # 3 x neighbors
            weighted_dist = self.alphas * square_dist # 3 x neighbors
            prob = np.sum(weighted_dist, axis=0).flatten()
            prob = np.exp(2 * np.array(prob))
            self.curr_location = np.random.choice(neighbors, p=prob / np.sum(prob))
        map.num_swarm_points[old_loc.x, old_loc.y] -= 1
        map.num_swarm_points[self.curr_location.x, self.curr_location.y] += 1
        return

    def check_detected(self, wrapper, map):
        if any(map.evader_detected):
            self.curr_location = map.evader_location
            self.was_evader_location = True
            self.prev_evader_location = (map.evader_location.x, map.evader_location.y)
            self.detected_since_evader = True
        elif wrapper.in_detection_zone(self.curr_location):
            if self.was_evader_location and self.detected_since_evader:
                self.curr_location = map.get_random_voxel_without_obstacle(self.curr_location, 3)
            else:
                swarm_locations = [s.curr_location for s in map.swarm]
                swarm_locations.remove(self.curr_location)
                self.curr_location = np.random.choice(swarm_locations) if np.random.random() < .99 else Location(*map.get_random_voxel_without_obstacle())
        else:
            self.detected_since_evader = False
