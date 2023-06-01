from __future__ import absolute_import, print_function
import numpy as np
from RRTTree import RRTTree
import sys
import time
from tqdm import tqdm

class RRTStarPlanner(object):

    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 10000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def compute_cost(self, node_id):
        root_id = self.tree.GetRootID()

        cost = 0
        node = self.tree.vertices[node_id]
        while node_id != root_id:
            parent_id = self.tree.edges[node_id]
            parent = self.tree.vertices[parent_id]
            cost += self.env.compute_distance(node, parent)

            node_id = parent_id
            node = parent

        return cost

    def Plan(self, start_config, goal_config, rad=10):
        # TODO: YOUR IMPLEMENTATION HERE
        start_id = self.tree.AddVertex(start_config, 0)
        for i in range(self.max_iter):
            x_rand = self.sample(goal_config)
            x_nearest_id, x_nearest = self.tree.GetNearestVertex(x_rand)
            x_new = self.extend(x_nearest, x_rand)
            if self.env.edge_validity_checker(x_nearest, x_new):
                X_near_ids, X_near = self.tree.GetNNInRad(x_new, rad)
                x_new_id = self.tree.AddVertex(x_new, 0) # 0 cost is fine cause it will done in connect and on line 44.
                x_min, x_min_id, cost_min = self.connect(x_nearest, x_nearest_id, x_new, x_new_id, X_near, X_near_ids)
                self.tree.AddEdge(x_min_id, x_new_id)
                self.tree.costs[x_new_id] = cost_min
                self.rewire(x_new, x_new_id, X_near, X_near_ids)
                if self.env.goal_criterion(x_new):
                    return self.find_path(start_config, start_id, x_new, x_new_id)

        return None

    def extend(self, x_near, x_rand):
        # TODO: YOUR IMPLEMENTATION HERE
        transition = x_rand - x_near

        # dist = self.env.compute_distance(x_near, x_rand)
        # if dist > self.eta:
        #     transition = transition * self.eta
        # norm = np.linalg.norm(direction)

        # step_size = self.eta
        # x_new = None
        # while step_size <= 1:
        #     x_new_test = x_near + step_size * transition
        #     if self.env.edge_validity_checker(x_near, x_new_test):
        #         x_new = x_new_test
        #     else:
        #         break
        #     step_size += self.eta
        # if x_new is not None:
        #     return x_new.astype(int)
        # return None

        x_new = x_near + self.eta * transition

        return x_new

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()
    
    def connect(self, x_nearest, x_nearest_id, x_new, x_new_id, X_near, X_near_ids):
        x_min = x_nearest
        x_min_id = x_nearest_id
        cost_min = self.compute_cost(x_nearest_id) + self.env.compute_distance(x_nearest, x_new)
        for i in range(len(X_near)):
            x_near = X_near[i]
            x_near_id = X_near_ids[i]
            if self.env.edge_validity_checker(x_near, x_new) and (self.compute_cost(x_near_id) + self.env.compute_distance(x_near, x_new) < cost_min):
                x_min = x_near
                x_min_id = x_near_id
                cost_min = self.compute_cost(x_near_id) + self.env.compute_distance(x_near, x_new)

        return x_min, x_min_id, cost_min

    def rewire(self, x_new, x_new_id, X_near, X_near_ids):
        for i in range(len(X_near)):
            x_near = X_near[i]
            x_near_id = X_near_ids[i]
            if self.env.edge_validity_checker(x_near, x_new) and (self.compute_cost(x_new_id) + self.env.compute_distance(x_new, x_near) < self.compute_cost(x_near_id)):
                self.tree.edges.pop(x_near_id)
                self.tree.AddEdge(x_new_id, x_near_id)
                self.tree.costs[x_near_id] = self.compute_cost(x_new_id) + self.env.compute_distance(x_new, x_near)

    def find_path(self, start, start_id, goal, goal_id):
        path = []
        s = goal_id
        dim = goal.shape[0]
        while s != start_id:
            path.append(self.tree.vertices[s].reshape(dim,).astype(int))
            s = self.tree.edges[s]
        path.append(start.reshape(dim,).astype(int))
        
        return np.array(path)
