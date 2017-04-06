from collections import defaultdict
from heapq import heappop, heappush
import numpy as np
import time
import sys

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.actions = dict()


    def Plan(self, start_config, goal_config):
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        # https://en.wikipedia.org/wiki/A*_search_algorithm
        print "Started A* planner."
        start_time = time.time()

        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        # import pdb; pdb.set_trace()
        # import IPython; IPython.embed()

        if self.visualize:
           self.planning_env.InitializePlot(goal_config)
        camefrom = dict()
        camefrom[start_id] = start_id

        dists = defaultdict(lambda: float('inf'))
        dists[start_id] = 0

        closed_set = dict() # key is node_id
        open_set = []   # Min-heap with items [score, node_id]
        heappush(open_set, (self.planning_env.ComputeHeuristicCost(start_id, goal_id), start_id))

        in_collision = dict()

        #Track number of nodes explored
        num_exp = 0

        # Remember everything is in node_id!!!
        while open_set:
            current = heappop(open_set)[1]
            num_exp = num_exp + 1

            if self.visualize:
                self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(camefrom[current]), self.planning_env.discrete_env.NodeIdToConfiguration(current))

            if (current == goal_id):
                break # reconstruct path and return
            closed_set[current] = True;

            neighbors = self.planning_env.GetSuccessors(current)

            for action in neighbors:
            	neighbor = action.footprint[-1]
            	neighbor = self.planning_env.discrete_env.ConfigurationToNodeId(neighbor)

                if (neighbor in closed_set or neighbor in in_collision):
                    continue

                # Check if neighbor is in collision
                if (neighbor not in in_collision):
                    # import pdb; pdb.set_trace()
                    neighbor_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor)
                    if (self.planning_env.RobotIsInCollisionAt(neighbor_config)):
                        in_collision[neighbor] = True
                        closed_set[neighbor] = True
                        continue

                dist2node = dists[current] + \
                            self.planning_env.ComputeDistance(current,neighbor)
                score =  5*self.planning_env.ComputeHeuristicCost(neighbor, goal_id) + dist2node

                check = [item for item in open_set if item[1] == neighbor]
                if neighbor not in closed_set.keys():
                    heappush(open_set, [score, neighbor])
                    closed_set[neighbor] = True
                elif (dist2node >= dists[neighbor]):
                    continue # This is not a better path
                elif (dist2node < dists[neighbor]) and check:
                    # This is a better path. Update score in open_set
                    open_set_index =  next(i for i, v in enumerate(open_set) if v[1] == neighbor)
                    open_set[open_set_index][0] = score

                self.actions[neighbor] = action
                camefrom[neighbor] = current
                dists[neighbor] = dist2node

        #If open_set ran out before reaching goal
        if (current!=goal_id):
            print "------\nPlanning failed. Couldn't find a valid path.\n------"
            sys.exit()

        # Reconstruct path as list
        plan = [self.actions[goal_id]]    # If planning succeeded, current==goal
        # if self.visualize:
        #     self.planning_env.PlotEdgeWithID(camefrom[goal_id], goal_id)
        plan_len = 0
        while (current != start_id):
            current = camefrom[current]
            # if self.visualize:
            #     self.planning_env.PlotEdgeWithID(camefrom[current], current)
            # plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current))
            plan.append(self.actions[current])
            plan_len += 1
        # plan.append(start_config)

        plan.reverse()
        end_time = time.time()

        # Convert plan to numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        plan_array = np.array([plan[0]])
        for i in range(1,len(plan)):
            plan_array = np.concatenate((plan_array,np.array([plan[i]])), axis=0)

        # Report statistics
        #print 'Final plan', plan
        print 'Path length: ', self.planning_env.ComputeDistancePath(plan)

        print 'Plan time: ', end_time - start_time
        print '# nodes expanded: ', num_exp

        return plan_array
