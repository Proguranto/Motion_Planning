from __future__ import absolute_import, print_function
import numpy as np
from RRTTree import RRTTree
import sys
import time

class RRTPlanner(object):

    def __init__(self, planning_env, bias = 0.05, eta = 1.0, max_iter = 10000):
        self.env = planning_env         # Map Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                # Goal Bias
        self.max_iter = max_iter        # Max Iterations
        self.eta = eta                  # Distance to extend

    def Plan(self, start_config, goal_config):
        # TODO: YOUR IMPLEMENTATION HERE
        came_from = {}
        dim = start_config.shape[0]
        came_from[tuple(start_config.reshape(dim,))] = None
        self.tree.AddVertex(start_config, cost=0)
        self.path = None
        goal_found = False
        goal = goal_config
        itr = 0
        if self.env.goal_criterion(start_config):
            goal_found = True
            goal = start_config
            return [tuple(start_config.reshape(dim,))]

        while itr < self.max_iter:
            x_rand = self.sample(goal=goal_config)
            x_near_id, x_near = self.tree.GetNearestVertex(x_rand)
            if tuple(x_rand) == tuple(x_near):
                continue
            x_new = self.extend(x_near=x_near, x_rand=x_rand)
            if x_new is not None:
                new_cost = self.tree.costs[x_near_id] + self.env.compute_distance(x_near, x_new)
                x_new_id = self.tree.AddVertex(x_new, cost=new_cost)
                self.tree.AddEdge(x_near_id, x_new_id)
                came_from[tuple(x_new.reshape(dim,))] = tuple(x_near.reshape(dim,))
                # print("x_new: ", x_new)
                # print("valid? ", self.env.dist_to_goal(x_new))
                if self.env.goal_criterion(x_new):
                    goal_found = True
                    goal = x_new
                    break
            itr += 1

        if goal_found:
            self.path = []
            s = tuple(goal.reshape(dim,))
            while s != tuple(start_config.reshape(dim,)):
                self.path.append(np.array(s).astype(int))
                s = came_from[s]
            self.path.append(tuple(start_config.reshape(dim,).astype(int)))
            self.path.reverse()              

        print("path: ", self.path)

        return np.array(self.path)

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

        if self.env.edge_validity_checker(x_near, x_new):
            return x_new
        return None

    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()
    