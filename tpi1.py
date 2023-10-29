#STUDENT NAME: Rostyslav Zabolotskyy
#STUDENT NUMBER: 117948

#DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *
import heapq
#using heapq helps with priority queing

class OrderDelivery(SearchDomain):

    def __init__(self,connections, coordinates):
        self.connections = connections
        self.coordinates = coordinates
        # ANY NEEDED CODE CAN BE ADDED HERE

    def actions(self,state):
        city = state[0]
        actlist = []
        for (C1,C2) in self.connections:
            if (C1==city):
                actlist += [(C1,C2)]
            elif (C2==city):
               actlist += [(C2,C1)]
        return actlist 

    def result(self,state,action):
        #IMPLEMENT HERE
        return action[1]
    def satisfies(self, state, goal):
        #IMPLEMENT HERE
        return state == goal

    def cost(self, action):
        for (C1,C2,D) in self.connections:
            if(C1,C2) == action or (C2,C1) == action:
                return D
        return float('inf')    

    def heuristic(self, city, goal_city):
        x1, y1 = self.coordinates[city]
        x2, y2 = self.coordinates[goal_city]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


 
class MyNode(SearchNode):

    def __init__(self,state,parent,depth,heuristic, cost):
        super().__init__(state,depth)
        self.state = state
        self.parent = parent
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic
        self.eval = cost + heuristic
        self.marked = False
        self.children = []
       
    def in_parent(self, newstate):
        if self.parent == None:
            return False
        if self.parent.state == newstate:
            return True
        return self.parent.in_parent(newstate)
    
    def __lt__(self, other):
        if self.eval == other.eval:
            return self.state < other.state
        return self.eval < other.eval    
    #compare eval values to order them for heap function
        #ADD HERE ANY CODE YOU NEED
        

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',maxsize=float('inf')):
        super().__init__(problem,strategy)
        self.problem = problem
        root = SearchNode(problem.initial,None)
        self.open_nodes = [root]
        root.depth = 0
        root.cost = 0
        root.heuristic = 0
        self.maxsize = maxsize
        self.terminals = 0
        self.non_terminals = 0
        
        #ADD HERE ANY CODE YOU NEED
    def get_path(self,node):
        if node.parent == None:
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return(path)
    def astar_add_to_open(self,lnewnodes):
        for newnode in lnewnodes:
            heapq.heappush(self.open_nodes, newnode)
        #IMPLEMENT HERE

    def search2(self):
        self.non_terminals = 0
        while self.open_nodes != []:
            node = heapq.heappop(self.open_nodes) #using heap to pop the first node with the lowest eval
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                return self.get_path(node)
            lnewnodes = []
            self.non_terminals += 1 
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                path_cost = node.cost + self.problem.domain.cost(node.state, a)
                #heuristic = self.problem.domain.heuristic(newstate, self.problem.goal)
                if not node.in_parent(newstate):
                        newnode = MyNode(newstate, node, node.depth+1,path_cost,0)
                        lnewnodes.append(newnode)
        self.astar_add_to_open(lnewnodes)
        self.manage_memory()
        return None
        #IMPLEMENT HERE
        
    def manage_memory(self):
        while len(self.open_nodes) > self.maxsize:
        # sort the queue based on the eval value in descending order
            self.open_nodes.sort(key=lambda node: node.eval, reverse=True)
         
        # find a node marked for deletion
        for idx, node in enumerate(self.open_nodes):
            # check if all its siblings are also marked for deletion
            siblings_marked = all([sibling.marked for sibling in node.parent.children if sibling is not node])

            if siblings_marked:
                parent = node.parent
                # remove the node and its siblings from the queue
                self.open_nodes = [n for n in self.open_nodes if n.parent is not parent]
                
                # update the parent eval value with the minimum eval value of its children
                min_child_eval = min([child.eval for child in parent.children])
                parent.eval = min_child_eval
                
                # add the parent back to the open_nodes
                heapq.heappush(self.open_nodes, parent)
                break
    def orderdelivery_search(self):
        return self.search2()
        #IMPLEMENT HERE

    # if needed, auxiliary methods can be added here


# If needed, auxiliary functions can be added here
