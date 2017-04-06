import numpy
import math
import time
import openravepy
import sys

class HerbEnvironment(object):
    def __init__(self, herb):
        self.robot = herb

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def GenerateRandomConfiguration(self):
        dof = len(self.robot.robot.GetActiveDOFIndices())
        lower_limits, upper_limits = self.robot.robot.GetActiveDOFLimits()

        config = [0] * dof

        collision = True
        while collision is True:
            for i in range(dof):
                config[i] = numpy.random.rand()*(upper_limits[i]-lower_limits[i]) + lower_limits[i]
            config=numpy.array(config).tolist()
            if self.RobotIsInCollisionAt(config) is False:
                collision = False

        return numpy.array(config)

    def ComputeDistance(self, start_config, end_config):
        dist = numpy.linalg.norm(start_config - end_config)
        return dist

    def ComputeDistancePath(self, path):
        dist = 0
        for i in range(len(path)-1):
            dist += self.ComputeDistance(path[i],path[i+1])
        return dist

    def Extend(self, start_config, end_config):

        goal_dist = self.ComputeDistance(start_config, end_config) #Calculate the distance to goal
        direction = (end_config - start_config)/goal_dist #Vector direction for robot's movement
        collision_flag = 0 #Initialize the collision flag as zero
        #No collisions in the start and end configurations since GenerateRandomConfiguration checks for collisions
        step_dist = goal_dist/10 #distance covered per waypoint to reach goal (arbitrary)
        waypoints = int(goal_dist/step_dist) #No. of waypoints to cover
        prev_config = start_config #initialize the return value in case of collision

        last_successful_config = start_config # Returns start point if it's impossible to extend

        #Start configuration to waypoint 1; check for collision, repeat for waypoint 2, 3 and so on till collision occurs/end_config is reached
        for ms in range (1, waypoints):
            # Make sure it we don't extend too far
            epsilon = 0.2

            if (self.ComputeDistance(start_config, last_successful_config)>epsilon):
                return last_successful_config

            new_config = numpy.sum([start_config,step_dist*ms*direction],axis=0) #Compute new configuration

            collision_flag = self.RobotIsInCollisionAt(new_config) #Check collision and update collision flag
            if collision_flag: #Check the boolean returned for collision
                last_successful_config = prev_config
                break
            else:
                last_successful_config = new_config
                prev_config = new_config
                continue

        return last_successful_config #returns last successful configuration

    def RobotIsInCollisionAt(self, point=None):
        """
        Call self.RobotIsInCollisionAt() to check collision in current state
        self.RobotIsInCollisionAt(numpy7darray) to check at another configuration
        """
        # Point should be a 7D numpy array with the configuration.
        # Leave empty if checking collision at current configuration.
        if point is None:
            in_collision = self.robot.robot.GetEnv().CheckCollision(self.robot.robot)
            self_collision = self.robot.robot.CheckSelfCollision()
            if (in_collision):
                # print "Collision"
                return True
            elif (self_collision):
                # print "Self collision"
                return True
            else:
                return False

        # Test configureations for table collision and self collision
        #goal_config = numpy.array([ 5.1, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
        #goal_config = numpy.array([ 5.1,  1.5,  2.0,  2.20,  0.00,  0.00,  0.00 ])

        # If checking collision in point other than current state, move robot
        #  to that point, check collision, then move it back.
        current_state = self.robot.robot.GetDOFValues()
        check_state = numpy.copy(current_state)
        dof = self.robot.robot.GetActiveDOFIndices()
        check_state[dof] = point

        self.robot.robot.SetDOFValues(check_state, range(len(current_state)), openravepy.KinBody.CheckLimitsAction.Nothing)  # move robot to state to check

        in_collision = self.robot.robot.GetEnv().CheckCollision(self.robot.robot)
        self_collision = self.robot.robot.CheckSelfCollision()

        # print check_state
        # print in_collision, self_collision
        # raw_input("stop")

        self.robot.robot.SetDOFValues(current_state, range(len(current_state)), openravepy.KinBody.CheckLimitsAction.Nothing)  # move robot back to current state

        if (in_collision):
            # print "collision"
            return True
        if (self_collision):
            # print "self collision"
            return True

        return False

    def RobotIsInCollisionOnLine(self, point1, point2):
        # Linear check
        n_points = 10
        increments = [float(i)/n_points for i in range(n_points+1)]
        for num in increments:
            check_at = point1 + num*(point2-point1)
            if(self.RobotIsInCollisionAt(check_at)):
                return True
        return False

    def ShortenPath(self, path, timeout=5.0):

        for node_index in range(len(path)-1):
            if self.robot.robotIsInCollisionOnLine(path[node_index], path[node_index+1]):
                print "-------------------------------------"
                print "The original path contains a collision."
                print "Collision is between:"
                print node_index, path[node_index]
                print node_index+1, path[node_index+1]
                print "-------------------------------------"
                sys.exit(0)

        print "Attempting to shorten path for ", timeout, " seconds."
        print "Initial distance ", self.ComputeDistancePath(path)
        print "Initial # of points ", len(path)

        # Shortcutting method
        start_time = time.clock()
        while (time.clock()-start_time)<timeout:
            # 1. Take two random points on the path
            #       pick random points from 0:end-1 on path
            rand1 = int(math.floor(numpy.random.random()*(len(path)-1)))
            rand2 = int(math.floor(numpy.random.random()*(len(path)-1)))

            if rand1>rand2:
                rand1, rand2 = rand2, rand1
            elif rand1==rand2:
                continue    # no point trying to shorten path on straight line

            #  random interpolation 0-1 between points and next points
            p1 = path[rand1] + numpy.random.random()*(path[rand1+1]-path[rand1])
            p2 = path[rand2] + numpy.random.random()*(path[rand2+1]-path[rand2])

            # 2. Check for collision free straight path between points
            if self.robot.robotIsInCollisionOnLine(p1, p2):
                # If the path has a collision, don't change path
                continue
            else:
                # If it is collision free, then replace everything in between
                new_path = [path[0]]
                for i in range(rand1+1):
                    new_path.append(path[i])
                new_path.append(p1)
                new_path.append(p2)
                for j in range(rand2+1,len(path)):
                    new_path.append(path[j])
                path = new_path

        print "Final distance: ", self.ComputeDistancePath(path)
        print "Final # of points ", len(path)
        return path
