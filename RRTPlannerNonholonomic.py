import numpy as np
from RRTTree import RRTTree
import time

class RRTPlannerNonholonomic(object):
    def __init__(self, planning_env, bias=0.05, max_iter=10000, num_control_samples=25):
        self.env = planning_env                 # Car Environment
        self.tree = RRTTree(self.env)
        self.bias = bias                        # Goal Bias
        self.max_iter = max_iter                # Max Iterations
        self.num_control_samples = 25           # Number of controls to sample

    def Plan(self, start_config, goal_config):
        plan = []
        plan_time = time.time()
        x_new_id = self.tree.GetRootID()
        # TODO: YOUR IMPLEMENTATION HERE
        start_id = self.tree.AddVertex(start_config)
        for i in range(self.max_iter):
            # print("stuck")
            x_rand = self.sample(goal_config)
            x_near_id, x_near = self.tree.GetNearestVertex(x_rand)
            if tuple(x_rand) == tuple(x_near):
                continue
            x_new, delta_t = self.extend(x_near, x_rand)
            if x_new is not None:
                # print("x_new: ", x_new)
                x_new_id = self.tree.AddVertex(x_new, self.tree.costs[x_near_id] + delta_t)
                self.tree.AddEdge(x_near_id, x_new_id)
                if self.env.goal_criterion(x_new, goal_config):
                    break
        # YOUR IMPLEMENTATION END HERE
        cost = 0
        while x_new_id != self.tree.GetRootID():
            cost+=self.tree.costs[x_new_id]
            plan.insert(1, self.tree.vertices[x_new_id])
            x_new_id = self.tree.edges[x_new_id]
        plan_time = time.time() - plan_time
        print("Cost: %f" % cost)
        if len(plan)>0:
            return np.concatenate(plan, axis=1)

    def extend(self, x_near, x_rand):
        """ Extend method for non-holonomic RRT

            Generate n control samples, with n = self.num_control_samples
            Simulate trajectories with these control samples
            Compute the closest closest trajectory and return the resulting state (and cost)
        """
        # TODO: YOUR IMPLEMENTATION HERE
        # U is the sampled actions
        U = [self.env.sample_action() for i in range(self.num_control_samples)]
        x_new = None
        min_dist = np.Inf
        best_delta_t = np.Inf
        for u in U:
            x_action, delta_t = self.env.simulate_car(x_near, x_rand, u[0], u[1])
            if x_action is None:
                continue
            curr_dist = self.env.compute_distance(x_action, x_rand)
            if curr_dist < min_dist:
                x_new = x_action
                min_dist = curr_dist
                best_delta_t = delta_t
        return x_new, best_delta_t
                
    def sample(self, goal):
        # Sample random point from map
        if np.random.uniform() < self.bias:
            return goal

        return self.env.sample()