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
        # sort graspset (in decreasing score order; best->worst)
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]

        # Go through sorted graspset and find one that works
        for i, grasp in enumerate(self.grasps_ordered):
            print "Checking grasp #", i, "/", len(self.grasps_ordered)

            #Find grasp configuration
            grasp_c = self.gmodel.getGlobalGraspTransform(grasp,collisionfree=True)

            # Test validity of grasp
            densityfn,samplerfn,bounds = self.robot.irmodel.computeBaseDistribution(grasp_c)
            arm_config, pose = self.test_grasp(grasp_c, densityfn,samplerfn,bounds)

            # Check if it works
            if arm_config is not None:
                base_pose = pose
                grasp_config = grasp_c
                break
        ###################################################################

        return base_pose, grasp_config


    def test_grasp(self,grasp_config,densityfn,samplerfn,bounds):
        success = False
        numfailures = 0
        N = 1
        with self.robot.GetEnv():
            while not success:
                if (numfailures>3):
                    break
                poses,jointstate = samplerfn(N)
                for pose in poses:
                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)

                    # validate that base is not in collision
                    if not self.robot.GetEnv().CheckCollision(self.robot):
                        q = self.manip.FindIKSolution(grasp_config,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            print "Success! Found good grasp."
                            self.robot.SetTransform(pose)
                            cur = self.robot.GetDOFValues()
                            cur[self.manip.GetArmIndices()] = q
                            self.robot.SetDOFValues(cur)
                            pose =  self.robot.GetTransform()
                            success = True
                        else:
                            numfailures += 1
        return q, pose

    def PlanToGrasp(self, obj):

        # Select a pose for the base and an associated ik for the arm
        print "Looking for good grasp and base pose..."
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Base pose or grasp_config is None.'
            exit()

        # Now plan to the base pose
        print 'Planning base trajectory'
        start_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        print 'Planning arm trajectory'
        start_config = np.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
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
              return None
