import numpy
import time
import random
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.1):

        # set epsilon to 1-3 for Herb

        if (self.planning_env.robot.robot.GetEnv().GetRobot('Herb2')  == self.planning_env.robot.robot):
            epsilon = 1

        self.tree = RRTTree(self.planning_env, start_config)
        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        m = start_config

        start = time.time()

        while (self.planning_env.ComputeDistance(m, goal_config) > epsilon):

            p = numpy.random.uniform(0,1)

            goal_sampling_prob = 0.6

            if p > goal_sampling_prob:

                randconf = self.planning_env.GenerateRandomConfiguration()

            else:
                randconf = goal_config

            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotPoint(randconf)

            v_id, v = self.tree.GetNearestVertex(randconf)

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
