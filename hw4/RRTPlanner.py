import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = .2): #.001
        if(self.planning_env.RobotIsInCollisionAt(start_config)):
            print "Start config is in collision!"
        if (self.planning_env.RobotIsInCollisionAt(goal_config)):
            print "Goal config is in collision!"

        tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
           self.planning_env.InitializePlot(goal_config)

        endFlag = False
        start_time = time.clock()
        while endFlag is False:
            # with self.planning_env.robot.robot.GetEnv():

            if np.random.rand() <= .2:
                config = goal_config
            else:
                config = self.planning_env.GenerateRandomConfiguration()

            vid, vertex = tree.GetNearestVertex(config)

            successConfig = self.planning_env.Extend(vertex, config)
            #raw_input("")
            if not np.array_equal(vertex,successConfig):
                if self.planning_env.RobotIsInCollisionOnLine(vertex,successConfig) is False:
                    tree.AddVertex(successConfig)
                    tree.AddEdge(vid,len(tree.vertices)-1)

            #if within a certain distance of goal & path to goal is collision free, end path searching
            if (self.planning_env.ComputeDistance(successConfig,goal_config) <= epsilon) and (self.planning_env.RobotIsInCollisionOnLine(successConfig,goal_config) is False):
                if self.planning_env.RobotIsInCollisionOnLine(vertex,successConfig) is False:
                    tree.AddEdge(len(tree.vertices)-1,len(tree.vertices))
                    tree.AddVertex(goal_config)
                    endFlag = True
                    print ("Found path")

            timeout = 30
            if (time.clock()-start_time) > timeout:
                print "RRT timed out"
                return 0

        plan.append(goal_config)
        idx = tree.edges[max(tree.edges)] #this is the parent of goal
        while idx:
            #import pdb;pdb.set_trace() #for debugging
            plan.append(tree.vertices[idx[0]])
            idx = tree.edges[idx[0]]
        #plan.append(start_config)

        plan = plan[::-1];
        print "Time: ",time.clock()-start_time

        return plan

    def FindMinIn(Q, dist):
        min_dist = 9999
        u = 0
        for index,node in enumerate(Q):
            if (dist[index] < min_dist):
                min_dist = dist[index]
                u = index
        return u
