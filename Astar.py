import heapq
import matplotlib.pyplot as plt
import numpy as np
import time
import math
import itertools

BLACK = np.array([0,0,0], dtype=np.uint8)
RED = np.array([255,0,0], dtype=np.uint8)
BLUE = np.array([0,0,255], dtype=np.uint8)
GREEN = np.array([0,255,0], dtype=np.uint8)

# Define a class to represent nodes in the search graph
class Node:
    def __init__(self, indices:np.array, g, h, parent, epsilon):
        self.indices = indices.flatten()
        self.g = g
        self.h = h
        self.parent = parent
        self.epsilon = epsilon

    def f(self):
        # TODO: YOUR IMPLEMENTATION HERE
        return self.g + self.epsilon * self.h

    # Define __lt__ for comparison with other nodes in the priority queue
    def __lt__(self, other):
        return self.f() < other.f()

class AstarPlanner:
    def __init__(self, env, epsilon:float):
        self.env = env
        self.epsilon = epsilon

        # variables for plotting
        image = np.array(self.env.c_space, dtype=np.uint8)
        image = np.repeat(image[:, :, None], repeats=3, axis=2)
        self.image = (1-image)*255

    def get_neighbors(self, node:Node):
        """ Returns all of the neighbouring nodes.
            @param node: Current node
            Return: list[Node]
        """
        neighbors = []
        indices = node.indices
        shape = self.env.c_space.shape

        # The neighboring nodes are:
        neighbors_indices = []
        for dim, idx in enumerate(indices):
            for offset in [-1, 1]:
                new_idx = list(indices)
                new_idx[dim] += offset
                if new_idx[dim] < 0:
                    continue
                elif new_idx[dim] >= shape[dim]:
                    continue
                neighbors_indices.append(new_idx)

        for indices in neighbors_indices:
            if self.env.c_space[tuple(indices)] == 1:
                continue
            neighbors.append(Node(np.array(indices), 0, 0, None, self.epsilon))

        return neighbors

    # Define the A* algorithm
    def Plan(self, start, goal):
        start = start.flatten()
        goal = goal.flatten()
        # print("start: ", start)
        # print("goal: ", goal)

        open_set = [Node(start, 0, 0, None, self.epsilon)]
        self.open_set_indices = {tuple(start)}
        self.closed_set_indices = set()
        self.path = []
        # TODO: YOUR IMPLEMENTATION HERE
        came_from = dict()
        came_from[tuple(start)] = None
        path_found = False
        while len(open_set) != 0:
            s = heapq.heappop(open_set)
            # print("s: ", s.indices)
            self.open_set_indices.remove(tuple(s.indices))
            self.closed_set_indices.add(tuple(s.indices))
            if tuple(s.indices) == tuple(goal):
                path_found = True
                break
            for successor in self.get_neighbors(s):
                new_cost = s.g + self.env.compute_distance(s.indices, successor.indices)
                # print("cost: ", new_cost)
                # print("successor", successor.indices)
                if tuple(successor.indices) not in self.closed_set_indices:
                    if tuple(successor.indices) not in self.open_set_indices:
                        # Not seen before so reset values.
                        successor.g, successor.h = new_cost, self.h(successor.indices)
                        heapq.heappush(open_set, successor)
                        self.open_set_indices.add(tuple(successor.indices))
                        came_from[tuple(successor.indices)] = tuple(s.indices)
                    elif successor.g > new_cost:
                        successor.g = new_cost
                        heapq.heapify(open_set)
                        came_from[tuple(successor.indices)] = tuple(s.indices)
        
        if path_found:
            s = tuple(goal)
            while s != tuple(start):
                self.path.append(s)
                s = came_from[s]
            self.path.append(tuple(start))
            self.path.reverse()
        else:
            self.path = None

        print("states explored: ", len(self.closed_set_indices))
        
        return self.path


    def h(self, start_config):
        """ Heuristic function for A*

            @param config: a [2 x 1] numpy array of state
        """
        # TODO: YOUR IMPLEMENTATION HERE
        return self.env.dist_to_goal(start_config)
    
    def plot(self, ):
        # Plot the elements in the open set
        for index in self.open_set_indices:
            self.image[index] = GREEN

        # Plot the elements in the closed set
        for index in self.closed_set_indices:
            self.image[index] = BLUE

        # Plot the elements in the path
        for index in self.path:
            self.image[index] = RED

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.im = self.ax.imshow(self.image)
        plt.show()
