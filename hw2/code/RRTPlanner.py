import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        


        plan.append(start_config)

        p= self.planning_env.Extend(start_config, goal_config)

        while (p is not goal_config):

            plan.append(p)

            random_pt = self.planning_env.GenerateRandomConfiguration()

            temp = self.planning_env.Extend(p, random_pt)

            self.planning_env.PlotEdge(p, temp)
            p = self.planning_env.Extend(temp, goal_config)

            self.planning_env.PlotEdge(temp, p)


        plan.append(goal_config)
        
        return plan
