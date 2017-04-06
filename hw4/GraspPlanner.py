import logging, openravepy
import numpy as np
import math
import os
import copy
import time
from DiscreteEnvironment import DiscreteEnvironment

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.manip = self.robot.GetActiveManipulator()
        self.end_effector = self.manip.GetEndEffector()
        self.env = self.robot.GetEnv()

        # For snapping to discrete grid
        lower_limits = [-5., -5., -np.pi]
        upper_limits = [5., 5., np.pi]
        resolution = [0.1, 0.1, np.pi/4.]
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

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


        self.base_config = base_pose
        ###################################################################

        #Uncomment to show grasp
        # self.robot.SetTransform(base_pose)
        # cur = self.robot.GetDOFValues()
        # original = cur
        # cur[self.manip.GetArmIndices()] = arm_config
        # self.robot.SetDOFValues(cur)
        # raw_input("Showing base pose and grasp config")
        # self.robot.SetDOFValues(original)

        # Convert base_pose to [x,y,theta] form
        T = self.robot.GetTransform()
        R = base_pose[0:3,0:3]
        axis_angle = openravepy.axisAngleFromRotationMatrix(R)
        yaw = axis_angle[2]
        base_pose = [base_pose[0,3], base_pose[1,3], yaw]
        print "Final pose: ", base_pose
        # raw_input("Check grasp")

        return base_pose, arm_config

    def snap_to_discrete(self, pose):
        """
        Snaps a given base pose to a discrete grid of 0.1, 0.1, pi/4 resolution
        Input will be in [x,y,z, q1, q2, q3, q4] form
        """

        T = self.robot.GetTransform()
        R = T[0:3,0:3]

        # import IPython
        # IPython.embed()

        # Convert quat to yaw
        axis_angle = openravepy.axisAngleFromRotationMatrix(R)
        yaw = axis_angle[2]

        # Feed to discrete_env: [x,y,theta]
        pose_3 = [pose[0], pose[1], yaw]
        grid_coord = self.discrete_env.ConfigurationToGridCoord(pose_3)
        discrete_pose = self.discrete_env.GridCoordToConfiguration(grid_coord)

        # Change back to original form with quat
        snapped_pose = openravepy.matrixFromAxisAngle( np.array( [0,0,discrete_pose[2]] ) )
        snapped_pose[0,3] = pose[0]
        snapped_pose[1,3] = pose[1]
        snapped_pose[2,3] = pose[2]

        return snapped_pose

    def test_grasp(self,grasp_config,densityfn,samplerfn,bounds):
        success = False
        numfailures = 0
        N = 1
        original_base = self.robot.GetTransform()
        with self.robot.GetEnv():
            while not success:
                if (numfailures>3):
                    break
                poses,jointstate = samplerfn(N)
                for pose in poses:
                    pose = self.snap_to_discrete(pose)

                    self.robot.SetTransform(pose)
                    self.robot.SetDOFValues(*jointstate)

                    # validate that base is not in collision
                    if not self.env.CheckCollision(self.robot):
                        q = self.manip.FindIKSolution(grasp_config,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                        if q is not None:
                            print "Success! Found good grasp."
                            self.robot.SetTransform(pose)
                            pose =  self.robot.GetTransform()
                            self.robot.SetTransform(original_base)
                            success = True
                        else:
                            numfailures += 1
        return q, pose

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

            METRIC = 2

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

###############################################################################

    def PlanToGrasp(self, obj):

        # Select a pose for the base and an associated ik for the arm
        print "Looking for good grasp and base pose..."
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Base pose or grasp_config is None.'
            exit()

        raw_input("enter to continue")
        self.robot.SetTransform(self.base_config)
        raw_input("Skipping base planning")

        # Now plan to the base pose
        # print 'Planning base trajectory'
        # start_pose = np.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        # base_plan = self.base_planner.Plan(start_pose, base_pose)
        # base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)
        #
        # print 'Executing base trajectory'
        # self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        print 'Planning arm trajectory'
        start_config = np.array(self.arm_planner.planning_env.robot.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.robot.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.robot.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()
