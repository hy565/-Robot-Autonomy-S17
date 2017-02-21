import numpy
import time
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
    
    #Detecting a if a line is intersect with the obstacle. If yes, return the previous coordinate
    #If not return the end point. 
    # def LineDetect(self,spt,ept,epsilon):
    #     ExtPoint = []
    #     for i in range(int(1.0/epsilon-2.0)):
    #         print i 
    #         print self.planning_env.Extend(spt,ept,epsilon*(i+1))
    #         ExtPoint.append(self.planning_env.Extend(spt,ept,epsilon*(i+1)))
    #         if ExtPoint[0] == None:
    #             return spt
    #         elif ExtPoint[i] == None:
    #             return ExtPoint[i-1]
    #     return ept

    def Plan(self, start_config, goal_config, epsilon = 0.3):
        
        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        
        # plan.append(start_config)

        m = start_config


        while (self.planning_env.ComputeDistance(m, goal_config) > epsilon):


            # NN = [] # generate a number of random configs, subject to certain constraints
            # while len(NN) < 11:
            #     rand = self.planning_env.GenerateRandomConfiguration()
            #     # if self.planning_env.ComputeDistance(rand, goal_config) < 3:
            #     # if self.planning_env.ComputeDistance(randconf, goal_config) < 3 :
            #     NN.append(rand)  
            # # get the distance of each config, and keep only the nearest one 
            
            # dNN = numpy.array([self.planning_env.ComputeDistance(j, m) for j in NN])

            # randconf = NN[numpy.argmin(dNN)]

            p = numpy.random.uniform(0,1)
            # print p
            goal_sampling_prob = 0.5

            if p > goal_sampling_prob:

                randconf = self.planning_env.GenerateRandomConfiguration()

            else:
                randconf = goal_config

            self.planning_env.PlotPoint(randconf)
            v_id, v = self.tree.GetNearestVertex(randconf)
            
            k = 0
            while (k < 10):
                m_prev = m
                m = self.planning_env.Extend(v, randconf)
                if m == None:
                    m = m_prev
                    # print "broke"
                    break
                
                elif (self.planning_env.ComputeDistance(m, randconf) < epsilon):
                    break

                else:
                    
                    # print k
                    k += 1
                    
            v_id, v = self.tree.GetNearestVertex(m)

            m_id = self.tree.AddVertex(numpy.array(m))
            self.tree.AddEdge(v_id, m_id)
            self.planning_env.PlotEdge(v, m)
            # time.sleep(0.5)
            # plan.append(m)

        goal_id = self.tree.AddVertex(goal_config)
        self.tree.AddEdge(m_id, goal_id)
        self.planning_env.PlotEdge(m, goal_config)


        currVertex = goal_id
        while (numpy.all(self.tree.vertices[currVertex] != start_config)):
            
            plan.insert(0, self.tree.vertices[currVertex])
            currVertex = self.tree.edges[currVertex]

        plan.insert(0, start_config)
        
        for i in range(len(plan)-1):
            self.planning_env.PlotEdgePlan(plan[i],plan[i+1])

        # plan.append(goal_config)
        self.path = plan
        print self.tree.edges
        print plan
        return plan
