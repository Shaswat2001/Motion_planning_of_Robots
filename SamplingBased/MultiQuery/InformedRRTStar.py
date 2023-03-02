import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list,check_nodes
from scipy.spatial.transform import Rotation as Rot
import math
import numpy as np
import matplotlib.pyplot as plt

class InformedRRTStar:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1,steering_const = 1,gamma = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.steering_const = steering_const
        self.goal_sample_rate = 0.1
        self.gamma = gamma
        self.x_sol = set()

        self.visited = [self.start]
        self.path = []
        
        self.c_min = calculate_distance(self.start,self.goal)
        self.x_centre = np.array([[(self.start.x+self.goal.x)/2.0],[(self.start.y+self.goal.y)/2.0],[0.0]])
        self.C = self.RotationToWorldFrame(self.c_min)

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        end_node = self.plan()
        self.plot.animate_rrt_star("Informed RRT*",self.visited,self.extract_path(end_node))

    def plan(self):

        # loops till size of tree is less than max_size
        for i in range(self.tree_size):
            
            c_best = self.getCbest()
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.Sample(c_best)
            # nearest node to sample_x
            near_x=self.Nearest(sample_x)
            # new node in the tree
            new_x=self.Steer(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.CheckEdgeCollision(near_x,new_x):

                index_table = self.Near(new_x)

                self.visited.append(new_x)

                if index_table:

                    self.create_new_path(new_x,index_table)
                    self.Rewire(new_x,index_table)

            if self.check_Node_goalRadius(new_x):

                self.x_sol.add(new_x)
            
            if i%200 == 0:
                print(i)

        node_idx = self.connectGoal()
        return self.visited[node_idx]
    
    def Sample(self,c_max):

        if c_max < math.inf:

            r1 = c_max/2.0
            r2 = math.sqrt(c_max**2 - self.c_min**2)/2.0
            L = np.diag([r1,r2,r2])

            while True:
                x_ball = self.SampleUnitNBall()
                x_rand = np.dot(np.dot(self.C, L), x_ball) + self.x_centre
                if self.graph.delta <= x_rand[0] <= self.graph.grid_size[0] - 1 - self.graph.delta and \
                        self.graph.delta <= x_rand[1] <= self.graph.grid_size[1] - 1 - self.graph.delta:
                    break
            x_rand = Node(x_rand[(0, 0)], x_rand[(1, 0)])

            return x_rand
        
        else:

            if np.random.random() > self.goal_sample_rate:
                    return self.graph.generate_random_node()

            return self.goal

    def Steer(self,x_sampNode,x_nearNode):
        '''
        Generates new Node in the grid

        Arguments:
        x_sampNode-- Node sampled from the grid
        x_nearNode-- Node nearest to x_sampNode
        nodeDist-- distance between x_nearNode and new Node

        Returns:
        x_new-- Object of class Node
        '''

        x_new=[0]*2
        # Coordinates of the nodes
        x_samp=x_sampNode.get_coordinates()
        x_near=x_nearNode.get_coordinates()

        # Checks if the distance between sampled and nearest node is less than nodeDist
        dist = calculate_distance(x_sampNode,x_nearNode)

        dx = x_samp[0] - x_near[0]
        dy = x_samp[1] - x_near[1]

        theta = math.atan2(dy,dx)
        dist = min(self.nodeDist, dist)

        x_new[0]=x_near[0] + dist*math.cos(theta)
        x_new[1]=x_near[1] + dist*math.sin(theta)

        newNode = Node(x_new[0],x_new[1])
        newNode.parent = x_nearNode
        # returns an object of class Node
        return newNode
    
    def RotationToWorldFrame(self,L):

        a1 = np.array([[(self.goal.x - self.start.x) / L],
                       [(self.goal.y - self.start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C


    def SampleUnitNBall(self):

         while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])
    
    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if calculate_distance(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False
        
    def connectGoal(self):
        
        dist_list = [calculate_distance(node,self.goal) for node in self.visited]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.steering_const]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.visited[i]) for i in node_index
                         if not self.graph.CheckEdgeCollision(self.visited[i], self.goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.visited) - 1
        
    def create_new_path(self,new_node,distance_table):

        cost = [self.total_cost(self.visited[i],new_node) for i in distance_table]

        index = distance_table[int(np.argmin(cost))]
        new_node.parent = self.visited[index]
        
    def Rewire(self,new_node,distance_table):

        for i in distance_table:

            nbr_node = self.visited[i]

            if self.total_cost(new_node,nbr_node) < self.cost(nbr_node):
                nbr_node.parent = new_node

    def Nearest(self,node):
        '''
        Finds nearest parent in the tree
        '''
        cost={}
        # Loops though all the nodes in the tree
        for i in self.visited:
            # distance between node and 'i'
            dist=calculate_distance(i,node)
            cost[i]=dist
        # Dict sorted with respect to distance
        cost=dict(sorted(cost.items(), key=lambda item: item[1]))
        #return closest node
        return list(cost.keys())[0]
    

    def cost(self,node):

        cost = 0

        if check_nodes(node,self.start):

            return 0
        
        if node.parent == None:
            return math.inf

        while node.parent != None:
            
            cost += calculate_distance(node,node.parent)
            node = node.parent

        return cost

    def total_cost(self,node,near_node):

        cost_node = self.cost(node)

        line_length = calculate_distance(node,near_node)

        return cost_node+line_length
        

    def Near(self,new_node):

        V = len(self.visited) + 1
        radius = 50*math.sqrt(math.log(V)/V)

        distance_table = [calculate_distance(new_node,nbr) for nbr in self.visited]
        index_dist_table = [i for i in range(len(distance_table)) if distance_table[i]<=radius and not self.graph.CheckEdgeCollision(new_node,self.visited[i])]

        return index_dist_table
    
    def getCbest(self):

        if len(self.x_sol) == 0:

            return math.inf
        
        cost = {}
        for i in self.x_sol:
            # distance between node and 'i'
            cost[i]=self.cost(i)
        # Dict sorted with respect to distance
        x_best = min(cost, key=cost.get)
        return cost[x_best]

    
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent
        bkt_list.append(node)

        return bkt_list

    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_dcm()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, ".b")
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)    