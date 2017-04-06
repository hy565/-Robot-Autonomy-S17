import operator
from collections import defaultdict

class RRTTree(object):

    def __init__(self, planning_env, start_config):

        self.planning_env = planning_env
        self.vertices = []
        self.vertices.append(start_config)
        self.edges = defaultdict(list) # use adjacency list

    def GetRootId(self):
        return 0

    def GetNearestVertex(self, config):

        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.ComputeDistance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]


    def AddVertex(self, config):
        vid = len(self.vertices)
        self.vertices.append(config)
        return vid

    # What do sid and eid stand for?
    # eid and sid are scalars (indices into self.vertices)
    # sid: node already in tree, eid: node that just got added
    def AddEdge(self, sid, eid):
        # Add nodes as neighbors to each other
        self.edges[eid].append(sid)
