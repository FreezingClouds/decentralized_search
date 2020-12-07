import numpy as np
from collections import OrderedDict

R = 5
max_intersection = 5

class Location(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance(self, location):
        return np.sqrt(np.square(location.x - self.x) + np.square(location.y - self.y))

class Agent(object):
    def __init__(self, id, x, y, update_every_k_steps = 5):
        self.id = id
        self.curr_location = Location(x, y)
        self.curr_path = []
        self.update_every_k_steps = update_every_k_steps
        self.counter = 0
        return

    def get_path(self, map, claimed_paths, evader_location=None, max_intersection = .5):
        self.counter += 1
        map.detected_evader(evader_location)

        if self.counter == self.update_every_k_steps or len(self.curr_path) == 0:
            # TODO: Change this to distance between different paths instead of intersections
            self.counter, valid_path = 0, False
            locations = [s.curr_location for s in map.swarm]
            boundaries = [((max(loc.x - R, 0), min(loc.x + R, map.x_max)),
                            (max(loc.y - R, 0), min(loc.y + R, map.y_max))) for loc in locations]
            densities = [np.sum(map.num_swarm_points[x_bound[0]: x_bound[1], y_bound[0]: y_bound[1]]) for x_bound, y_bound in boundaries]
            locations_to_densities = dict(zip(locations, densities))
            locations_to_densities = iter(sorted(locations_to_densities.items(), key=lambda item: item[1], reverse=True))
            while not valid_path:
                new_location, density = next(locations_to_densities)
                if self.curr_location == new_location:
                    continue
                path = map.get_path(self.curr_location, new_location)
                intersect_prev_paths = [set(path).intersection((set(prev_path))) for prev_path in claimed_paths]
                too_similar_to_prev_path = [len(intersect) > max_intersection for intersect in intersect_prev_paths]
                if not any(too_similar_to_prev_path):
                    valid_path = True
                    self.curr_path = path
        return self.curr_path

class SwarmPoint(Agent):
    def __init__(self, x, y, alpha1, alpha2, alpha3):
        super(SwarmPoint, self).__init__(-1, x, y, update_every_k_steps=1)
        self.alphas = (alpha1, alpha2, alpha3)
        return

    def move_one_step(self, wrapper, map):
        map.num_swarm_points[self.curr_location.x, self.curr_location.y] -= 1
        if map.evader_detected:
            self.curr_location = map.evader_location
        elif map.in_detection_zone(self.curr_location):
            swarm_locations = [s.curr_location for s in map.swarm]
            swarm_locations.remove(self.curr_location)
            self.curr_location = np.random.choice(swarm_locations)
        else:  # evader not detected and swarmpoint not in detection zone
            neighbors = map.get_voxel_neighbors(self.curr_location)
            neighbor_values = [self.compute_weighted_sum(neighbor_location, wrapper) for neighbor_location in neighbors]
            best_neighbor = np.argmin(neighbor_values)
            choice = neighbors[best_neighbor]
            self.curr_location = choice

        map.num_swarm_points[self.curr_location.x, self.curr_location.y] += 1
        return

    def compute_weighted_sum(self, location, wrapper):
        return sum([p.curr_location.distance(location) * self.alphas[i] for i, p in enumerate(wrapper.pursuers)])
