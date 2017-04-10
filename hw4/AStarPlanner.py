import time
import numpy
from collections import deque

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()
        self.actions = dict()
        self.costs = dict()
        self.dist_so_far = dict()


    def Plan(self, start_config, goal_config):
        hGain = 100 #gain for heuristics
        #print hGain

        # if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
        self.planning_env.InitializePlot(goal_config)

        # q = deque([])
        q = []
        visited = []
        plan = []
        # expanded = []

        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)


        self.nodes[start_node] = (start_node, None)
        self.costs[start_node] = self.planning_env.ComputeHeuristicCost(start_node, goal_node)
        self.dist_so_far[start_node] = 0
        

        print start_config, goal_config
        print start_node,goal_node
        print "Trying to get to: ", self.planning_env.discrete_env.NodeIdToConfiguration(goal_node)
        print "Lower Limits: ", self.planning_env.lower_limits
        print "Upper Limits  ", self.planning_env.upper_limits
        q.append(start_node)
        visited.append(start_node)
        while len(q) is not 0:


            # print "Queue:", q
            # print "Visited: ", visited

            #sort the queue
            # print [self.costs[i] for i in q] 
            q = sorted(q, key=self.costs.__getitem__, reverse=False)    #sort q based on costs
            node = q.pop(0)


            print "Curr Node:", node, " Goal Node", goal_node
            # print "Nodes away: ", abs(goal_node - node)
            print "Heuristic :", hGain*self.planning_env.ComputeHeuristicCost(node,goal_node)
            print "Path Cost: ", self.dist_so_far[node]


            last_node_config = self.planning_env.discrete_env.NodeIdToConfiguration(self.nodes[node][0])
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(node)
            # idx = self.planning_env.discrete_env.NodeIdToGridCoord(node)[2]
            if self.visualize:
<<<<<<< HEAD
                    self.planning_env.PlotEdge(last_node_config,node_config)
                    
                    # self.planning_env.PlotActionFootprints(idx)

                #print "Node popped: ",node
            if node == goal_node:
                print "goal found!"
                curr_node = node

                while curr_node is not start_node:
                    # plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(curr_node))
                    plan.insert(0, self.nodes[curr_node][1])
                    #print "Current Node: ", curr_node
                    curr_node = self.nodes[curr_node][0]

                # plan.insert(0,self.actions[start_node])
                
                # print "Plan Length: ", len(plan)*self.planning_env.resolution

                print len(plan)
                return plan
                    
            successors = self.planning_env.GetSuccessors(node)
            # print "Successors: ",successors
            
            for action in successors:

                neighbor = node_config + action.footprint[-1]
                neighbor[2] = action.footprint[-1][2]
                # neighbor[0] = neighbor[0] + node_config[0]
                # neighbor[1] = neighbor[1] + node_config[1]
                neighbor = self.planning_env.discrete_env.ConfigurationToNodeId(neighbor)
                print "Current neighbor:", neighbor
                new_dist = self.dist_so_far[node] + self.planning_env.ComputeDistance(node, neighbor)
                # print "new_dist:", new_dist, " dist_so_far: ", self.dist_so_far[node]
                if neighbor not in visited: #or new_dist < self.dist_so_far[neighbor]:
                    self.dist_so_far[neighbor] = new_dist
                    q.append(neighbor)
                    visited.append(neighbor)

                    print neighbor, "added to queue"
                    ##assign cost = distance + heuristic
                    self.costs[neighbor] = self.dist_so_far[neighbor] + hGain*self.planning_env.ComputeHeuristicCost(neighbor,goal_node)
                    self.nodes[neighbor] = (node, action)
                    # self.actions[neighbor] = action
                    # print self.actions

            # expanded.append(node)
            #print "Plan: ",plan
            #print "Visited: ",visited
            #time.sleep(0.5)

        return None


=======
                # self.planning_env.PlotEdgeWithID(camefrom[current], current)
                self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(camefrom[current]), self.planning_env.discrete_env.NodeIdToConfiguration(current))

            if (np.all(self.planning_env.discrete_env.NodeIdToGridCoord(current)[0:2] == self.planning_env.discrete_env.NodeIdToGridCoord(goal_id)[0:2])):
                self.a = True
                print "Almost there!"
                print self.planning_env.discrete_env.NodeIdToGridCoord(goal_id), self.planning_env.discrete_env.NodeIdToGridCoord(current)
                self.r = (self.planning_env.discrete_env.NodeIdToGridCoord(goal_id)[2] -  self.planning_env.discrete_env.NodeIdToGridCoord(current)[2])
                print "Rotations left: ", self.r
                self.actions[current] = action
                break
                
            if (current == goal_id):
                #camefrom[neighbor] = current
                #dists[neighbor] = dist2node
                break # reconstruct path and return

            closed_set[current] = True;
            actions = self.planning_env.GetSuccessors(current)
            curr_config = self.planning_env.discrete_env.NodeIdToConfiguration(current)

            for action in actions:
            	neighbor = action.footprint[-1][0:2] + curr_config[0:2]
                neighbor = np.concatenate((neighbor, np.array([action.footprint[-1][2]])), axis=0)

            	neighbor = self.planning_env.discrete_env.ConfigurationToNodeId(neighbor)

                if (neighbor in closed_set or neighbor in in_collision):
                    continue

                # Check if neighbor is in collision
                if (neighbor not in in_collision):
                    neighbor_config = self.planning_env.discrete_env.NodeIdToConfiguration(neighbor)
                    if (self.planning_env.RobotIsInCollisionAt(neighbor_config)):
                        in_collision[neighbor] = True
                        closed_set[neighbor] = True
                        continue

                dist2node = dists[current] + \
                            self.planning_env.ComputeDistance(current,neighbor)
                score =  5*self.planning_env.ComputeHeuristicCost(neighbor, goal_id) + dist2node

                check = [item for item in open_set if item[1] == neighbor]
                if neighbor not in closed_set.keys():
                    heappush(open_set, [score, neighbor])
                    closed_set[neighbor] = True
                elif (dist2node >= dists[neighbor]):
                    continue # This is not a better path
                elif (dist2node < dists[neighbor]) and check:
                    # This is a better path. Update score in open_set
                    open_set_index = next(i for i, v in enumerate(open_set) if v[1] == neighbor)
                    open_set[open_set_index][0] = score

                self.actions[neighbor] = action
                camefrom[neighbor] = current
                dists[neighbor] = dist2node

        #If open_set ran out before reaching goal
        if (current!=goal_id) and not self.a:
            print "Planning failed. Couldn't find a valid path."
            import IPython
            IPython.embed()
            sys.exit()

        # Reconstruct path as list
        plan = []

        for i in range(abs(self.r)):
            if self.r < 0: #need to turn counter-clockwise
                plan.append(self.planning_env.actions[0][2])

            else: # turn clockwise
                plan.append(self.planning_env.actions[0][1])
        # if self.visualize:
        #     self.planning_env.PlotEdgeWithID(camefrom[goal_id], goal_id)
        plan_len = 0
        while (current != start_id):
            current = camefrom[current]
            try:
                plan.append(self.actions[current])
            except:
                break
            plan_len += 1
            # if self.visualize:
            #     self.planning_env.PlotEdgeWithID(camefrom[current], current)
            # plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(current))

            # print plan_len
        #plan.append(self.actions[start_id])
        #plan.append(self.actions[current])
        #plan_len += 1
        plan.reverse()
        end_time = time.time()

        # Convert plan to numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        # plan_array = np.array([plan[0]])
        # for i in range(1,len(plan)):
            # plan_array = np.concatenate((plan_array,np.array([plan[i]])), axis=0)

        # Report statistics
        print 'Final plan', plan
        # print 'Path length: ', self.planning_env.ComputeDistancePath(plan)

        print 'Plan time: ', end_time - start_time
        print '# nodes expanded: ', num_exp
        print np.asarray(plan).shape
        return plan
>>>>>>> 334fb1bec860564a525f1b1da0a8864405dee23a
