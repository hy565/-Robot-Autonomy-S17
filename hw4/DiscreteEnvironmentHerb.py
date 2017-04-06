import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        #config = [0] * self.dimension
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config

    def ConfigurationToGridCoord(self, config):
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        #coord = [0] * self.dimension
        coord = numpy.floor((numpy.array(config)-numpy.array(self.lower_limits))/self.resolution).tolist()
        return coord

    def GridCoordToConfiguration(self, coord):
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        #config = [0] * self.dimension
        config = numpy.array(coord)*self.resolution + self.resolution/2 + numpy.array(self.lower_limits)
        return config

    def GridCoordToNodeId(self,coord):
        # This function maps a grid coordinate to the associated
        # node id
        node_id = coord[0]
        for idx in range(1, self.dimension):
            node_id = node_id + coord[idx]*numpy.prod(self.num_cells[0:idx])
        return node_id

    def NodeIdToGridCoord(self, node_id):
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension
        temp_id = node_id
        for idx in range(self.dimension-1, 0, -1):
            coord[idx] = numpy.floor(temp_id/(numpy.prod(self.num_cells[0:idx])))
            temp_id = temp_id%(numpy.prod(self.num_cells[0:idx]))
        coord[0] = temp_id
        return coord
