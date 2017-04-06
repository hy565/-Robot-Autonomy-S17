import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import itertools
import copy

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance

            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize

        # Add one more config that snaps the last point in the footprint to the center of the cell
        gc = self.discrete_env.ConfigurationToGridCoord(config)
        snapped_config = self.discrete_env.GridCoordToConfiguration(gc)

        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)
        print("start: ", start_config)
        print("last: ", config)
        print("snapped: ", snapped_config)
        print '-'

        return footprint

    # def PlotActionFootprints(self, idx):
    def PlotActionFootprints(self):
        actions = self.actions
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        # print actions
        for idx in range(int(self.discrete_env.num_cells[2])):
            for action in actions[idx]:
                # print action.footprint[-1]
                xpoints = [config[0] for config in action.footprint]
                ypoints = [config[1] for config in action.footprint]

                pl.plot(xpoints, ypoints, 'k')
                pl.plot(action.footprint[-1][0], action.footprint[-1][1],'.r')

        pl.ion()
        pl.show()

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()

        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)


        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            # print grid_coordinate
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)
            original_pose = self.herb.robot.GetTransform()

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process

            #Set of Actions:
            ##Construct action objects

            #Move Forward from current configuration/pose:
            #Diagonal Step:
            if not (idx % 2 == 0):
                # print 'Diagonal bud'
                ControlF = Control(1,1,numpy.sqrt(2)*0.4)
                FootprintF =  self.GenerateFootprintFromControl(start_config, ControlF)
                ActionF =  Action(ControlF, FootprintF)
            else:
                # print 'Straight ahead'
                ControlF =  Control(1, 1, 0.4)  #ul,ur,time
                FootprintF =  self.GenerateFootprintFromControl(start_config, ControlF)
                ActionF =  Action(ControlF, FootprintF)
            #Turn CW by pi/4:
            ControlCW =  Control(1, -1, numpy.pi/4.)
            FootprintCW =  self.GenerateFootprintFromControl(start_config, ControlCW)
            ActionCW =  Action(ControlCW, FootprintCW)
            #Turn CCW by pi/4:
            ControlCCW =  Control(-1, 1, numpy.pi/4.)
            FootprintCCW =  self.GenerateFootprintFromControl(start_config, ControlCCW)
            ActionCCW =  Action(ControlCCW, FootprintCCW)

            self.actions[idx] = [ActionF, ActionCW, ActionCCW]



    def GetSuccessors(self, node_id):

        successors = []

        current_grid = self.discrete_env.NodeIdToGridCoord(node_id)#Convert node_id to grid coordinate
        current_orientation = current_grid[2]#Get the current orientation
        current_config = self.discrete_env.NodeIdToConfiguration(node_id)

        # print "Now Expanding: ", current_grid, current_config

        for action in self.actions[current_orientation]:
            collision = False    #Initialize collision flag as false
            for footprint in action.footprint:
                test_config = copy.deepcopy(current_config)
                test_config += footprint

                if test_config[2] > numpy.pi:
                    test_config[2] -= 2.*numpy.pi
                if test_config[2] < -numpy.pi:
                    test_config[2] += 2.*numpy.pi

                test_coord = self.discrete_env.ConfigurationToGridCoord(test_config)
                # print "Candidate successor: ", test_coord,

                # print numpy.array([0]*self.discrete_env.dimension)
                # print numpy.array(self.discrete_env.num_cells)
                if (numpy.any(test_coord[0:2] < numpy.array([0]*2))) or (numpy.any(test_coord[0:2] >=  numpy.array(self.discrete_env.num_cells[0:2]))):
                    print test_coord
                    collision =  True
                    break

            if not collision:
                successors.append(action) #Last footprint corresponds to node id and controls are embedded in the action

        # print successors
        return successors


    def ComputeDistance(self, start_id, end_id):
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = numpy.linalg.norm(start_config[0:2] - end_config[0:2]) #Returns an array of len = len(config) --- Euclidean distance, since the robot can turn
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = numpy.linalg.norm(start_config[0:2] - goal_config[0:2]) #Returns an array of len = len(config) --- Distance and Heuristic must be of the same form, with some weights
        #The robot should move towards the goal position, then adjust its orientation
        return cost

    def RobotIsInCollisionAt(self, point=None):
        """
        Call self.RobotIsInCollisionAt() to check collision in current state
             self.RobotIsInCollisionAt(numpy2darray) to check at another point
        """

        # Point should be a 2D numpy array with the configuration.
        # Leave empty if checking collision at current point
        if point is None:
            return self.robot.GetEnv().CheckCollision(self.robot)

        # If checking collision in point other than current state, move robot
        #  to that point, check collision, then move it back.
        # point = self.discrete_env.NodeIdToConfiguration(point)
        with self.robot.GetEnv():
            current_state = self.robot.GetTransform()
            # print current_state
            check_state = numpy.copy(current_state)
            T = numpy.copy(current_state)

            T = openravepy.matrixFromAxisAngle([0, 0, point[2]])
            # print T[0:2]
            # check_state[:2,3] = point[0:2]
            T[:2,3] = point[0:2]
            # T[0][3] = point[0]
            # T[1][3] = point[1]
            # print T
            self.robot.SetTransform(T)
            in_collision = self.robot.GetEnv().CheckCollision(self.robot)
            self.robot.SetTransform(current_state)  # move robot back to current state
        return in_collision


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        self.cnt = 0
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')


        pl.ion()
        pl.show()



    def PlotEdge(self, sconfig, econfig, render_interval = 1):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=0.5)


        self.cnt += 1
        # render_interval = 100/self.resolution
        # render_interval = 1
        if self.cnt > render_interval:

            pl.draw()
            self.cnt = 0
