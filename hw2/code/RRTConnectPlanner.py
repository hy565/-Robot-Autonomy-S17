import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

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
        
    def Endgame(self, ftree_v,rtree_v,epsilon):

        d = 0;
        # print "range ftree_v = ",range(len(ftree_v))
        # print "range rtree_v = ",range(len(rtree_v))

        for f_vid in range(len(ftree_v)):
            # print "fvid = ",f_vid
            for r_vid in range(len(rtree_v)):
                # print "rvid = ",r_vid
                d = self.planning_env.ComputeDistance(ftree_v[f_vid],rtree_v[r_vid])
                if d < epsilon:
                    return 1,f_vid,r_vid

        return 0,f_vid,r_vid
        



    def Plan(self, start_config, goal_config, epsilon = 0.2):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        CheckEndGame = 0
        fcnct = 0
        rcnct = 0
        qhat = [0,0]
        k = 10

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan.append(start_config)

        ftree.AddVertex(start_config)
        rtree.AddVertex(goal_config)

        mf = start_config
        mr = goal_config

        while CheckEndGame==0:

            qhat = self.planning_env.GenerateRandomConfiguration()

            ftree.GetNearestVertex(qhat)
            rtree.GetNearestVertex(qhat)
            self.planning_env.PlotPoint(qhat)

            fv_id, fv = ftree.GetNearestVertex(qhat)
            rv_id, rv = rtree.GetNearestVertex(qhat)

            #Extend for ftree
            for i in range(k):
                mf_prev = mf
                mf = self.planning_env.Extend(fv,qhat)

                if mf == None:
                    mf = mf_prev
                    print "broke"
                    break
                #elif()

            fv_id, fv = ftree.GetNearestVertex(mf)
            mf_id = ftree.AddVertex(mf)
            ftree.AddEdge(fv_id,mf_id)
            self.planning_env.PlotEdge(fv,mf)


            #Extend for rtree
            for j in range(k):
                mr_prev = mr
                mr = self.planning_env.Extend(rv,qhat)

                if mr == None:
                    mr = mr_prev
                    print "broke"
                    break

            rv_id, rv = rtree.GetNearestVertex(mr)
            mr_id = rtree.AddVertex(mr)
            rtree.AddEdge(rv_id,mr_id)
            self.planning_env.PlotEdge(rv,mr)

            CheckEndGame,fcnct,rcnct = self.Endgame(ftree.vertices,rtree.vertices,epsilon)

        

        self.planning_env.PlotEdge(ftree.vertices[fcnct], rtree.vertices[rcnct])



        plan.append(goal_config)
        
        print plan
        return plan
