import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

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
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space




        plan.append(start_config)
        

        ept = goal_config
        spt = self.LineDetect(start_config,ept,epsilon)
        self.planning_env.PlotEdge(start_config,spt)

        while (self.LineDetect(spt,goal_config,epsilon) is not goal_config):


            NN = [] # generate a number of random configs, subject to certain constraints


            while len(NN) < 11: 
                randconf = self.planning_env.GenerateRandomConfiguration()
                # if self.planning_env.ComputeDistance(randconf, goal_config) < self.planning_env.ComputeDistance(spt, goal_config):
                # if randconf[0] > spt[0]:
                # if self.planning_env.ComputeDistance(randconf, goal_config) < 3 :
                NN.append(randconf)  


            # get the distance of each config, and keep only the nearest one 
            
            dNN = numpy.array([self.planning_env.ComputeDistance(j, spt) for j in NN])

            randconfig = NN[numpy.argmin(dNN)]

            ept = self.LineDetect(spt,randconfig,epsilon)

            # ept = self.LineDetect(spt,self.planning_env.GenerateRandomConfiguration(),epsilon)
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
        
        print plan
        return plan
