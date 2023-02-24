import math
from Visualize import Visualize
from Nodes import Node,calculate_distance

class FMTStar:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1,gamma = 1,sample_node = 100):

        self.start = start
        self.start.cost = 0
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.gamma = gamma
        self.sample_node = sample_node

        self.CLOSED = []
        self.V_open = set()
        self.V_open.add(self.start)
        self.V_unvisited = []
        self.V = []
        self.r_n = self.gamma*math.sqrt(math.log(self.sample_node)/self.sample_node)

        self.path = []

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        self.sample_nodes()

        z = self.start
        Visited = []
        while not self.check_Node_goalRadius(z):
            
            V_open_new = set()
            near_nodes = self.Near(self.V_unvisited,z)
            Visited.append(z)

            for x in near_nodes:
                Y_near = self.Near(self.V_open, x)
                cost_list = {y: y.cost + self.cost(y, x) for y in Y_near}
                y_min = min(cost_list, key=cost_list.get)

                if not self.graph.check_edge_CollisionFree(y_min, x):
                    x.parent = y_min
                    V_open_new.add(x)
                    self.V_unvisited.remove(x)
                    x.cost = y_min.cost + self.cost(y_min, x)

            self.V_open.update(V_open_new)
            self.V_open.remove(z)
            self.CLOSED.append(z)

            if not self.V_open:
                print("open set empty!")
                break

            cost_open = {y: y.cost for y in self.V_open}
            z = min(cost_open, key=cost_open.get)

        self.plot.animate_fmt_star("FMT* Search",self.V,Visited[1: len(Visited)],self.extract_path(z))

    def sample_nodes(self):


        while len(self.V) <= self.sample_node:

            new_node = self.graph.generate_random_node()

            if not self.graph.check_node_CollisionFree(new_node):

                self.V.append(new_node)

        self.V_unvisited = self.V.copy()
        self.V_unvisited.append(self.goal)
        self.V.append(self.start)

    def Near(self,node_list,z):

        return [node for node in node_list if calculate_distance(node,z)<=self.r_n]
    
    def cost(self,start,end):

        if self.graph.check_edge_CollisionFree(start, end):

            return math.inf
        
        return calculate_distance(start,end)
    
    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if calculate_distance(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False
    
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent

        bkt_list.append(node)
        
        return bkt_list
        

