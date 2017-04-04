import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import itertools
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
        # self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        # self.resolution = resolution
        # self.ConstructActions()

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
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])

        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')

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
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #



    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        current_grid = self.discrete_env.NodeIdToGridCoord(node_id)#Convert node_id to grid coordinate
        neighbor_gen = list((itertools.product([-1,0,1], repeat=self.discrete_env.dimension)))
        neighbor_gen.remove(tuple([0]*self.discrete_env.dimension))
        candidates = [np.array(current_grid) + n for n in np.array(neighbor_gen)]
        successors = [c for c in candidates
                        if (np.all(c >= np.array([0]*self.discrete_env.dimension))) and \
                           (np.all(c <  np.array(self.discrete_env.num_cells)))]
        #Check for collisions
        successors = [self.discrete_env.GridCoordToNodeId(s) for s in successors
                        if not self.RobotIsInCollisionAt(self.discrete_env.GridCoordToConfiguration(s))]

        return successors


    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist = np.linalg.norm(start_config - end_config) #Returns an array of len = len(config) --- Euclidean distance, since omnidirectional
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        cost = np.linalg.norm(start_config - goal_config) #Returns an array of len = len(config) --- Distance and Heuristic must be of the same form, with some weights
        return cost

    def RobotIsInCollisionAt(self, point=None):
        """
        Call self.RobotIsInCollisionAt() to check collision in current state
             self.RobotIsInCollisionAt(np2darray) to check at another point
        """

        # Point should be a 2D np array with the configuration.
        # Leave empty if checking collision at current point
        if point is None:
            return self.robot.GetEnv().CheckCollision(self.robot)

        # If checking collision in point other than current state, move robot
        #  to that point, check collision, then move it back.
        current_state = self.robot.GetTransform()

        check_state = np.copy(current_state)
        check_state[:2,3] = point
        self.robot.SetTransform(check_state)

        in_collision = self.robot.GetEnv().CheckCollision(self.robot)
        self.robot.SetTransform(current_state)  # move robot back to current state
        return in_collision
