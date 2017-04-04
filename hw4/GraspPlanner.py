import logging, openravepy
import numpy as np
import math
import os
import copy
import time

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.manip = self.robot.GetActiveManipulator()
        self.end_effector = self.manip.GetEndEffector()
        self.env = self.robot.GetEnv()

    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            print("Generating grasp set.")
            self.gmodel.autogenerate()
        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle

        cur_config = self.robot.GetTransform()
        obj_config = obj.GetTransform()
        robot_x = cur_config[0,3]

        while not self.robot.GetEnv().CheckCollision(self.robot):
            robot_x += 0.1
            robot_config = np.array([ [1, 0, 0, robot_x],
                                      [0, 1, 0, obj_config[1,3]],
                                      [0, 0, 1, 0],
                                      [0, 0, 0, 1]])
            self.robot.SetTransform(robot_config)

        robot_x = 0.25 #TODO take this out
        robot_config = np.array([ [1, 0, 0, robot_x-0.2],
                                  [0, 1, 0, obj_config[1,3]],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])
        self.robot.SetTransform(robot_config)
        print("Got as close to table as possible")

        # robot_config = np.array([ [-1, 0, 0, robot_x],
        #                           [0, -1, 0, obj_config[1,3]],
        #                           [0, 0, 1, 0],
        #                           [0, 0, 0, 1]])



        # TODO evaluate quality of grasps
        #For best grasps, test for reachability and find base pose
        for i, grasp in enumerate(self.grasps):
            print "Checking grasp #", i, "/", len(self.grasps)

            #Find grasp configuration
            grasp_config = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
            obj_config = obj.GetTransform()
            grasp_config[0,3] = obj_config[0,3]
            grasp_config[1,3] = obj_config[1,3]
            grasp_config[2,3] = obj_config[2,3]
            # self.show_grasp(grasp)
            # import IPython
            # IPython.embed()
            # grasp_config = self.manip.GetTransform()
            # grasp_config[0] = 0.01
            # q = self.manip.FindIKSolution(grasp_config, filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
            # print q
            # if q:
            #     import IPython
            #     IPython.embed()


            # # # TODO Test validity of grasp
            densityfn,samplerfn,bounds = self.robot.irmodel.computeBaseDistribution(grasp_config)
            valid_grasp = self.test_grasp(grasp_config, densityfn,samplerfn,bounds)

        ###################################################################

        return base_pose, grasp_config


    def test_grasp(self,grasp_config,densityfn,samplerfn,bounds):
        goals = []
        N = 1
        numfailures = 0

        while len(goals) < N:
            if (numfailures>2):
                break
            poses,jointstate = samplerfn(N-len(goals))
            for pose in poses:
                # import IPython
                # IPython.embed()
                self.robot.SetTransform(pose)
                self.robot.SetDOFValues(*jointstate)
                # validate that base is not in collision
                if not self.robot.GetEnv().CheckCollision(self.robot):
                    q = self.manip.FindIKSolution(grasp_config,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                    if q is not None:
                        print "Success!"
                        import IPython
                        IPython.embed()
                    else:
                        numfailures += 1
                        print "numfailures = ", numfailures
        return goals

    def show_grasp(self, grasp, delay=3):
        "displays the grasp"
        with openravepy.RobotStateSaver(self.gmodel.robot):
            time.sleep(0.1) # let viewer update?
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp)
                contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                self.gmodel.robot.GetController().Reset(0)
                self.gmodel.robot.SetDOFValues(finalconfig[0])
                self.gmodel.robot.SetTransform(finalconfig[1])
                self.env.UpdatePublishedBodies()
                time.sleep(delay)
            except openravepy.planning_error,e:
              print 'bad grasp!',e

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()

    def eval_grasp(self, grasp):
        """
        function to evaluate grasps
        returns a score, which is some metric of the grasp
        higher score should be a better grasp
        """
        with self.robot:
          #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
          try:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

            obj_position = self.gmodel.target.GetTransform()[0:3,3]
            # for each contact
            G = np.zeros((6,len(contacts))) #the wrench matrix
            for i, c in enumerate(contacts):
              pos = c[0:3] - obj_position
              dir = -c[3:] #this is already a unit vector

              G[0:3,i] = pos
              G[3:6,i] = np.cross(pos,dir)

            rankG = np.linalg.matrix_rank(G)
            if (rankG<6):
                return 0.0
            # print('Rank of G is:', rankG)

            METRIC = 3

            if (METRIC==1):
                # Metric 1: minimum singular value
                U, s, V = np.linalg.svd(G, full_matrices=True)
                # S = np.diag(s)
                score = s[-1] # sigma_min
            elif (METRIC==2):
                # Metric 2: Isotropy
                U, s, V = np.linalg.svd(G, full_matrices=True)
                score = s[-1]/s[0]
            elif (METRIC==3):
                # Metric 3: Volume of grasp map
                score =  np.sqrt(np.linalg.det(np.dot(G,np.transpose(G))))
                if (math.isnan(score)):
                    print("We'll set this value to zero.")
                    score = 0.0
            else:
                print("Please choose metric.")
                return
            return score

          except openravepy.planning_error,e:
              print("Planning error")
              return 9999
