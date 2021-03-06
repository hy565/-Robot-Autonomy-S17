import numpy
import time
import random
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    # def ShortenPath(self, path, timeout=5.0):
        
    #     # 
    #     # TODO: Implement a function which performs path shortening
    #     #  on the given path.  Terminate the shortening after the 
    #     #  given timout (in seconds).
    #     #
    #     # path = [list(i) for i in path]
    #     # new_path = list(path)
    #     new_path = [list(i) for i in path]

    #     start = time.time()

    #     # epsilon = 0.1
    #     epsilon = 3
        

    #     while (time.time()-start < timeout):

    #         first, second = random.sample(new_path, 2)
    #         first_idx = new_path.index(first)
    #         second_idx = new_path.index(second)

    #         if abs(first_idx - second_idx) == 1:
    #             continue

    #         # print first, second
    #         s = first
    #         connected = 0

    #         # while (time.time()-start < timeout and not connected):
    #         #     # print time.time()-start
    #         #     s_prev = s
    #         #     # s = self.planning_env.Extend(s, second, delta=0.1)
    #         #     s = self.planning_env.Extend(numpy.array(s), numpy.array(second))
    #         #     if s == None:
    #         #         break

    #         #     elif (self.planning_env.ComputeDistance(s, second) < epsilon):
    #         #         connected = True
    #         #         print "Connected!: ",

    #         s = self.planning_env.Extend(numpy.array(s), numpy.array(second), 100)


    #         # if connected:
    #         if self.planning_env.ComputeDistance(s, second) < epsilon:

    #             if first_idx > second_idx:
    #                 a = first_idx
    #                 first_idx = second_idx
    #                 second_idx = a

    #             print first_idx, second_idx

    #             # new_path = new_path[0:first_idx] + new_path[second_idx:]
    #             if first_idx == 0 and second_idx == len(new_path):
    #                 new_path[:] = [new_path[0]] + [new_path[-1]]
                
    #             elif first_idx == 0:
    #                 new_path[:] = [new_path[0]] + new_path[second_idx:]

    #             elif second_idx == len(new_path):
                        
    #                 new_path[:] = new_path[:1+first_idx] + [new_path[-1]]
    #             else:
    #                 new_path[:] = new_path[:1+first_idx] + new_path[second_idx:]
   

    #             print new_path, "new length: ", len(new_path), "  old length: ", len(path)

    #         else:
    #             continue
                


    #     return numpy.array(new_path)

    def Plan(self, start_config, goal_config, epsilon = 0.1):



        # set epsilon to 1-3 for Herb

        if (self.planning_env.robot.GetEnv().GetRobot('Herb2')  == self.planning_env.robot):
            epsilon = 1

        print "epsilon: ", epsilon



        
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

        start = time.time()

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
            # goal_sampling_prob = self.planning_env.p
            goal_sampling_prob = 0.6

            if p > goal_sampling_prob:

                randconf = self.planning_env.GenerateRandomConfiguration()

            else:
                randconf = goal_config

            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotPoint(randconf)
            
            v_id, v = self.tree.GetNearestVertex(randconf)
            
            # k = 0
            # while (k < 10):
            #     m_prev = m
            #     m = self.planning_env.Extend(v, randconf)
            #     if m == None:
            #         m = m_prev
            #         # print "broke"
            #         break
                
            #     elif (self.planning_env.ComputeDistance(m, randconf) < epsilon):
            #         break

            #     else:
                    
            #         # print k
            #         k += 1
                    
            m = self.planning_env.Extend(v, randconf, 30)

            v_id, v = self.tree.GetNearestVertex(m)

            m_id = self.tree.AddVertex(numpy.array(m))

            self.tree.AddEdge(v_id, m_id)
            
            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotEdge(v, m)
            # time.sleep(0.5)
            # plan.append(m)

        goal_id = self.tree.AddVertex(goal_config)
        self.tree.AddEdge(m_id, goal_id)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.PlotEdge(m, goal_config)


        currVertex = goal_id
        while (numpy.all(self.tree.vertices[currVertex] != start_config)):
            
            plan.insert(0, self.tree.vertices[currVertex])
            currVertex = self.tree.edges[currVertex]

        plan.insert(0, start_config)
        
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.ShowPlan(plan)

        plan_star = self.planning_env.ShortenPath(plan)

        if len(plan_star) < len(plan):
            print "shorter path found!"
            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.ShowPlan(plan_star, 'g')


        # plan.append(goal_config)
        self.path = plan
        self.path_star = plan_star

        # print self.tree.edges
        # print plan_star
        elapsed = time.time() - start

        print elapsed
        print self.getCost(plan)

        print len(self.tree.vertices)
        # print self.getCost(numpy.array(plan_star))
        # print len(plan_star)
        return plan


    def getCost(self,plan):

        sumdist = 0
        for i in range(len(plan)-1):
            
            dist = self.planning_env.ComputeDistance(plan[i], plan[i+1])
            sumdist += dist
        return sumdist