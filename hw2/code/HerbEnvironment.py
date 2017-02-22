import numpy
import time
import random

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
	bodies =  self.robot.GetEnv().GetBodies()
        config = [0] * len(self.robot.GetActiveDOFIndices())
	lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
	collision = True
	while(collision):	
		config = numpy.array(numpy.random.uniform(lower_limits,upper_limits,len(self.robot.GetActiveDOFIndices())))
		self.robot.SetActiveDOFValues(config)
		collision = (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(bodies[1],bodies[0]))
        return numpy.array(config)


    
    def ComputeDistance(self, start_config, end_config):
        
	return numpy.sum(numpy.square(start_config - end_config))
        


    def Extend(self, start_config, end_config, delta=30):
    	
	bodies =  self.robot.GetEnv().GetBodies()
	collision = (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(bodies[1],bodies[0]))
	if(collision):
	    return None
	config = [0]*len(start_config)
	for j in range(len(start_config)):
	    config[j] = start_config[j]		
	sampling_rate = 100;
	diff = end_config - start_config
    	diff /= sampling_rate
	# delta = 30
	while(not collision and delta>0):
	    config += diff
	    delta -=1
            self.robot.SetActiveDOFValues(config) 
            collision = (self.robot.CheckSelfCollision() or self.robot.GetEnv().CheckCollision(bodies[1],bodies[0])) 
        if(collision):
	    config -= diff
            self.robot.SetActiveDOFValues(config)
	return config	
        pass
        
    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        # path = [list(i) for i in path]
        # new_path = list(path)
        new_path = [list(i) for i in path]

        start = time.time()

        # epsilon = 0.1
        epsilon = 1
        

        while (time.time()-start < timeout):

            first, second = random.sample(new_path, 2)
            first_idx = new_path.index(first)
            second_idx = new_path.index(second)

            if abs(first_idx - second_idx) == 1:
                continue

            # print first, second
            s = first
            connected = 0

            # while (time.time()-start < timeout and not connected):
            #     # print time.time()-start
            #     s_prev = s
            #     # s = self.planning_env.Extend(s, second, delta=0.1)
            #     s = self.planning_env.Extend(numpy.array(s), numpy.array(second))
            #     if s == None:
            #         break

            #     elif (self.planning_env.ComputeDistance(s, second) < epsilon):
            #         connected = True
            #         print "Connected!: ",

            s = self.Extend(numpy.array(s), numpy.array(second), 100)


            # if connected:
            if self.ComputeDistance(s, second) < epsilon:

                if first_idx > second_idx:
                    a = first_idx
                    first_idx = second_idx
                    second_idx = a

                print first_idx, second_idx

                # new_path = new_path[0:first_idx] + new_path[second_idx:]
                if first_idx == 0 and second_idx == len(new_path):
                    new_path[:] = [new_path[0]] + [new_path[-1]]
                
                elif first_idx == 0:
                    new_path[:] = [new_path[0]] + new_path[second_idx:]

                elif second_idx == len(new_path):
                        
                    new_path[:] = new_path[:1+first_idx] + [new_path[-1]]
                else:
                    new_path[:] = new_path[:1+first_idx] + new_path[second_idx:]
   

                print new_path, "new length: ", len(new_path), "  old length: ", len(path)

            else:
                continue
                


        return numpy.array(new_path)