import numpy, operator
import time
import random
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

        
    def Endgame(self, ftree_v,rtree_v,epsilon):

        d = 0;
        # print "range ftree_v = ",range(len(ftree_v))
        # print "range rtree_v = ",range(len(rtree_v))

        for f_vid in range(len(ftree_v)):
            # print "fvid = ",f_vid
            for r_vid in range(len(rtree_v)):
                # print "rvid = ",r_vid
                d = self.planning_env.ComputeDistance(ftree_v[f_vid],rtree_v[r_vid])
                # print d
                if d < epsilon:
                    return 1,f_vid,r_vid

        return 0,f_vid,r_vid
        



    def Plan(self, start_config, goal_config, epsilon = 0.1):

        # set epsilon to 1-3 for Herb

        if (self.planning_env.robot.GetEnv().GetRobot('Herb2')  == self.planning_env.robot):
            epsilon = 1

        print "epsilon: ", epsilon
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        CheckEndGame = 0
        fcnct = 0
        rcnct = 0
        qhat = [0,0]
        k = 10

        start = time.time()

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        #plan.append(start_config)

        ftree.AddVertex(start_config)
        goal_id = rtree.AddVertex(goal_config)

        mf = start_config
        mr = goal_config

        while CheckEndGame==0:

            qhat = self.planning_env.GenerateRandomConfiguration()

            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotPoint(qhat)

            fv_id, fv = ftree.GetNearestVertex(qhat)
            rv_id, rv = rtree.GetNearestVertex(qhat)

            # #Extend for ftree
            # for i in range(k):
            #     mf_prev = mf
            #     mf = self.planning_env.Extend(fv,qhat)

            #     if mf == None:
            #         mf = mf_prev
            #         print "broke"
            #         break
            #     #elif()

            mf = self.planning_env.Extend(fv,qhat)
            fv_id, fv = ftree.GetNearestVertex(mf)
            mf_id = ftree.AddVertex(mf)
            ftree.AddEdge(fv_id,mf_id)
            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotEdge(fv,mf)


            # #Extend for rtree
            # for j in range(k):
            #     mr_prev = mr
            #     mr = self.planning_env.Extend(rv,qhat)

            #     if mr == None:
            #         mr = mr_prev
            #         print "broke"
            #         break
            mr = self.planning_env.Extend(rv,qhat)
            rv_id, rv = rtree.GetNearestVertex(mr)
            mr_id = rtree.AddVertex(mr)
            rtree.AddEdge(rv_id,mr_id)
            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.PlotEdge(rv,mr)

            CheckEndGame,fcnct,rcnct = self.Endgame(ftree.vertices,rtree.vertices,epsilon)
            if CheckEndGame == 1:
                fcnct_id = rtree.AddVertex(ftree.vertices[fcnct])
                rtree.AddEdge(rcnct,fcnct_id)

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):

            self.planning_env.PlotEdge(ftree.vertices[fcnct], rtree.vertices[rcnct])

        #Generate Path

        

        #Grow a path from f connect point to start point
        currVertex = fcnct
        while (numpy.all(ftree.vertices[currVertex] != start_config)):
            
            plan.insert(0, ftree.vertices[currVertex])
            currVertex = ftree.edges[currVertex]

        plan.insert(0, start_config)


        #Grow a path from the common to the goal
        currVertex = fcnct_id
        while (numpy.all(rtree.vertices[currVertex] != goal_config)):
        
            plan.append(rtree.vertices[currVertex])
            currVertex = rtree.edges[currVertex]

        plan.append(goal_config)



        #Plot the path
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.ShowPlan(plan)

        plan_star = self.planning_env.ShortenPath(plan)

        
        if len(plan_star) < len(plan):
            print "shorter path found!"
            if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
                self.planning_env.ShowPlan(plan_star, 'g')


        #plan.append(goal_config)
        elapsed = time.time() - start



        print elapsed
        print self.getCost(plan)

        print len(ftree.vertices) + len(rtree.vertices)
        # print self.getCost(numpy.array(plan_star))
        self.path_star = plan_star
        self.path = plan        
        # print plan
        return plan

    def getCost(self,plan):

        sumdist = 0
        for i in range(len(plan)-1):
            
            dist = self.planning_env.ComputeDistance(plan[i], plan[i+1])
            sumdist += dist
        return sumdist


