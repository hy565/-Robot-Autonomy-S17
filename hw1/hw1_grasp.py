#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)

curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()


class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    self.order_grasps()

    #order grasps with noise
    # self.order_grasps_noisy()


  def openrave_init(self):
    "the usual initialization for openrave"
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW1 Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

  def problem_init(self):
    """
    problem-specific initialization - load target and grasp module
    Grasps are generated at end of this function
    """
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

    #change the location so it's not under the robot
    T = self.target_kinbody.GetTransform()
    T[0:3,3] += np.array([0.5, 0.5, 0.5])
    self.target_kinbody.SetTransform(T)
    self.env.AddKinBody(self.target_kinbody)

    # create a grasping module
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)

    # if you want to set options, e.g. friction
    options = openravepy.options
    options.friction = 0.1
    if not self.gmodel.load():
      self.gmodel.autogenerate(options) # This is where grasps are first tested
    self.graspindices = self.gmodel.graspindices
    self.grasps = self.gmodel.grasps


  def order_grasps(self):
    "order the grasps - call eval grasp on each, set the 'performance' index, and sort"
    self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered

    for i,grasp in enumerate(self.grasps_ordered):
      print('Evaluated ' + str(i) + '/' + str(len(self.grasps_ordered)))
      grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)

    # sort! (in decreasing score order; best->worst)
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]

    # show top grasps
    print(order[1:10])

    for grasp in self.grasps_ordered:
        self.show_grasp(grasp)
        print(grasp[self.graspindices.get('performance')])
#        raw_input("Hit enter to move on to next grasp.")

  def order_grasps_noisy(self):
  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp
      print("entered noisy")
      self.grasps_ordered_noisy = self.grasps.copy() #you should change the order of self.grasps_ordered_noisy
    # Set the score with your evaluation function (over random samples) and sort

      for i,grasp in enumerate(self.grasps_ordered_noisy):
        print("evalauting grasp " + str(i))
        orig_score = self.eval_grasp(grasp)
        trials = [orig_score]
        for i in range(5):
          noisy_grasp = self.sample_random_grasp(grasp)
          trials.append(self.eval_grasp(noisy_grasp)) # add noise

        trials = np.array(trials)
        noise_score = np.mean(trials)
        grasp[self.graspindices.get('performance')] = noise_score # assign combined score

      # sort!
      order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]]) # why [0] here?
      order = order[::-1] # reverse to descending order
      self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]

      for grasp in self.grasps_ordered_noisy:
        print(grasp[self.graspindices.get('performance')])
        self.show_grasp(grasp)

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

        METRIC = 1

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

        # print(score)
        # self.show_grasp(grasp)

        return score

      except openravepy.planning_error,e:
        #you get here if there is a failure in planning
        #example: if the hand is already intersecting the object at the initial position/orientation
        return  0.00 # TODO you may want to change this

      #heres an interface in case you want to manipulate things more specifically
      #NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



  def sample_random_grasp(self, grasp_in):
    """
    given grasp_in, create a new grasp which is altered randomly
    you can see the current position and direction of the grasp by:
      # grasp[self.graspindices.get('igrasppos')]
      # grasp[self.graspindices.get('igraspdir')]
    """
    grasp = grasp_in.copy()

    # contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

    #sample random position
    RAND_DIST_SIGMA = 0.005  #TODO you may want to change this
    pos_orig = grasp[self.graspindices['igrasppos']]    #3x1
    #set a random position
    new_pos = pos_orig + np.random.normal(0, RAND_DIST_SIGMA,3)

    #sample random orientation
    RAND_ANGLE_SIGMA = np.pi/36 #TODO you may want to change this
    dir_orig = grasp[self.graspindices['igraspdir']]    #3x1
    roll_orig = grasp[self.graspindices['igrasproll']]  #1x1
    #set the direction and roll to be random
    new_dir = dir_orig + np.random.normal(0, RAND_ANGLE_SIGMA,3)
    new_roll = roll_orig + np.random.normal(0,RAND_ANGLE_SIGMA,1)

    grasp[self.graspindices['igrasppos']] = new_pos
    grasp[self.graspindices['igraspdir']] = new_dir
    grasp[self.graspindices['igrasproll']] = new_roll

    return grasp


  def show_grasp(self, grasp, delay=30):
    "displays the grasp"
    with openravepy.RobotStateSaver(self.gmodel.robot):
      with self.gmodel.GripperVisibility(self.gmodel.manip):
        time.sleep(0.1) # let viewer update?
        try:
          with self.env:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
            #if mindist == 0:
            #  print 'grasp is not in force closure!'
            contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
            self.gmodel.robot.GetController().Reset(0)
            self.gmodel.robot.SetDOFValues(finalconfig[0])
            self.gmodel.robot.SetTransform(finalconfig[1])
            self.env.UpdatePublishedBodies()
            time.sleep(delay)
        except openravepy.planning_error,e:
          print 'bad grasp!',e

if __name__ == '__main__':
  robo = RoboHandler()
  #time.sleep(10000) #to keep the openrave window open

#def order_grasps_noisy(self):
#    "order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp"
#    self.grasps_ordered_noisy = self.grasps.copy() #you should change the order of self.grasps_ordered_noisy
#    # set the score with your evaluation function (over random samples) and sort
#    NUM_NOISY = 10

#    for i,grasp in enumerate(self.grasps_ordered_noisy):
#        # print('Evaluated ' + str(i) + '/' + str(len(self.grasps_ordered_noisy)))
#        perturbed_scores = np.zeros((NUM_NOISY,1))
#        for j in range(NUM_NOISY):
#            noisy_grasp = self.sample_random_grasp(grasp)
#            perturbed_scores[j] = self.eval_grasp(noisy_grasp)

#        # Is mean the best way to do this?
#        grasp[self.graspindices.get('performance')] = np.mean(perturbed_scores) # Try min?
#        print("score for " + str(i) + ": " + str(np.mean(perturbed_scores)))

#    # sort! (in decreasing score order; best->worst)
#    order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
#    order = order[::-1]
#    self.grasps_ordered_noisy = self.grasps_ordered_noisy[order] # reorder in decreasing score order

# Theo's
