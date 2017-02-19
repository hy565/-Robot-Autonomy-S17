import numpy
import matplotlib.pyplot as pl

class SimpleEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def CheckInvalidConfig(self, config):
        limits = self.limits()
        if (config[0]>limits[1] and config[0]<limits[0] and config[1]>limits[3] and config[1]<limits[2]):
            return True
        else:
            return False
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration
        #
        

        
        # limits = self.limits()
        collision = 1

        while(collision):

            config = numpy.random.uniform(lower_limits,upper_limits,2)
            # collision = self.CheckInvalidConfig(config)

            with self.robot.GetEnv():
                T = self.robot.GetTransform()
                orig_T = T
                T[0][3] = config[0]
                T[1][3] = config[1]
                self.robot.SetTransform(T)

            if self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetRobots()[0]):
                self.robot.SetTransform(orig_T)
                collision = 1
            else:
                self.robot.SetTransform(orig_T)
                collision = 0



        # self.PlotPoint(config)        
        return config

    def ComputeDistance(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        distance = numpy.sqrt((start_config[0]-end_config[0])**2+(start_config[1]-end_config[1])**2)

        #pass
        return distance


    def limits(self):
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            x_upper_lim = bb.pos()[0] + bb.extents()[0]
            x_lower_lim = bb.pos()[0] - bb.extents()[0]
            y_upper_lim = bb.pos()[1] + bb.extents()[1]
            y_lower_lim = bb.pos()[1] - bb.extents()[1]
        return [x_upper_lim,x_lower_lim,y_upper_lim,y_lower_lim]

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        # limits = self.limits()
        # if (start_config[0]>limits[1] and start_config[0]<limits[0] and start_config[1]>limits[3] and start_config[1]<limits[2]):
        #     return None

        # elif (end_config[0]>limits[1] and end_config[0]<limits[0] and end_config[1]>limits[3] and end_config[1]<limits[2]):
        #     return None
        
        delta = 0.25

        # if self.CheckInvalidConfig(start_config) or self.CheckInvalidConfig(end_config):
        #     return None

        # else:
        dx = (end_config[0]-start_config[0])*delta
        dy = (end_config[1]-start_config[1])*delta
        ExConfig = [0]*2
        ExConfig[0] = start_config[0] + dx
        ExConfig[1] = start_config[1] + dy
        if self.CheckInvalidConfig(ExConfig):
            return None
        else:
            
            with self.robot.GetEnv():
                T = self.robot.GetTransform()
                orig_T = T
                T[0][3] = ExConfig[0]
                T[1][3] = ExConfig[1]
                self.robot.SetTransform(T)

                if self.robot.GetEnv().CheckCollision(self.robot.GetEnv().GetRobots()[0]):
                    self.robot.SetTransform(orig_T)
                    return None
                else:
                    self.robot.SetTransform(orig_T)
                    return ExConfig





    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

    def PlotPoint(self, config):
        pl.plot(config[0], config[1], 'bx')
        pl.show()