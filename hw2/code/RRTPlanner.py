import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
    
    #Detecting a if a line is intersect with the obstacle. If yes, return the previous coordinate
    #If not return the end point. 
    def LineDetect(self,spt,ept,epsilon):
        ExtPoint = []
        for i in range(int(1.0/epsilon-2.0)):
            print i 
            print self.planning_env.Extend(spt,ept,epsilon*(i+1))
            ExtPoint.append(self.planning_env.Extend(spt,ept,epsilon*(i+1)))
            if ExtPoint[0] == None:
                return spt
            elif ExtPoint[i] == None:
                return ExtPoint[i-1]
        return ept

    def Plan(self, start_config, goal_config, epsilon = 0.01):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        plan.append(start_config)

        ept = goal_config
        spt = self.LineDetect(start_config,ept,epsilon)
        self.planning_env.PlotEdge(start_config,spt)

        while (self.LineDetect(spt,goal_config,epsilon) is not goal_config):

            ept = self.LineDetect(spt,self.planning_env.GenerateRandomConfiguration(),epsilon)
            self.planning_env.PlotEdge(spt,ept)
            spt = ept
            plan.append(spt)
        self.planning_env.PlotEdge(ept,goal_config)

        # ept = goal_config
        # spt = start_config
        # self.planning_env.PlotEdge(start_config,self.LineDetect(start_config,goal_config,epsilon))

        # while (self.LineDetect(spt,goal_config,epsilon) is not goal_config):
        #     spt = self.LineDetect(spt,ept,epsilon)
        #     ept = self.planning_env.GenerateRandomConfiguration()
        #     self.planning_env.PlotEdge(spt,ept)
        #     plan.append(spt)
        # self.planning_env.PlotEdge(ept,goal_config)
        
        plan.append(goal_config)
        
        return plan
