import heapq, random
from collections import deque

class Node:
    def __init__(self, name):
        self.name = name
        self.successor = []
        self.cost = 0
        self.predecessor = ()
class Neighbour():
    def __init__(self, neighbour, weight, action, loc):
        self.neighbour = neighbour
        self.weight = weight
        self.action = action
        self.loc = loc


class PriorityQueue:
    """
      Implements a priority queue data structure. Each inserted item
      has a priority associated with it and the client is usually interested
      in quick retrieval of the lowest-priority item in the queue. This
      data structure allows O(1) access to the lowest-priority item.
    """
    def  __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (_, _, item) = heapq.heappop(self.heap)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

    def contains(self, item):
        for i in self.heap:
            #print (i[0],i[1],i[2])
            if i[2].name == item.name:
                return True
        return False

    def updatekey(self, item, prio):
        for i in self.heap:
            if i[2].name == item.name:
                l = list(i)
                l[0] = prio
                i = l
                return True
        return False

class AStar:
    def __init__(self, heuristic, startnode, goal):
        self.openlist = PriorityQueue()
        self.closedlist = set()
        self.heuristic = heuristic
        self.startnode = startnode
        self.goalnode = goal
        self.path = 0

    def astar(self):
        print ("A-Star")
        self.openlist.push(self.startnode, self.startnode.cost)
        #self.openlist.push(self.startnode.successor[0].neighbour, 3)
        print ("Start :",self.startnode.name)
        #print (self.openlist.contains(self.startnode.successor[0].neighbour))
        while True:
            """ Node with min cost """
            currentNode = self.openlist.pop()

            """ Goal found? """
            if currentNode == self.goalnode:
                print("Goal:",currentNode.name," found")
                return self.goalnode

            self.closedlist.add(currentNode)
            #print(currentNode in self.closedlist)
            self.expandNode(currentNode)

            if self.openlist.isEmpty():
                break
        return False
    def expandNode(self, currentNode):
        for successor in currentNode.successor:
            #print (successor.neighbour.name)
            if successor in self.closedlist:
                continue
            """the distance from start to the neighbor through current + weight of the edge from current to neighbor """
            tentative_g = currentNode.cost + successor.weight

            if self.openlist.contains(successor.neighbour) and tentative_g >= successor.neighbour.cost:
                continue
            successor.neighbour.predecessor = currentNode, successor
            #print (successor.neighbour.name, currentNode.name, successor.neighbour.predecessor.name)
            successor.neighbour.cost = tentative_g

            f = tentative_g + self.heuristic[successor.neighbour]
            #print (f)
            successor.neighbour.cost = f
            if self.openlist.contains(successor.neighbour):
                self.openlist.updatekey(successor.neighbour, f)
            else:
                self.openlist.push(successor.neighbour, f)

def action_sequenz(direction_seq, current_loc):
    print("CALC ACTION SEQ")
    print(current_loc)
    actions = []
    for tuple in direction_seq:
        print("action",tuple[0], "dist: ", tuple[1])
        goal = tuple[0]
        print("current loc: ", current_loc , "dist loc: ", goal)
        if current_loc == 'south':
            if goal == 'west':
                actions.append('left')
            elif goal == 'north':
                actions.append('forward')
            elif goal == 'east':
                actions.append('right')
        if current_loc == 'east':
            if goal == 'west':
                actions.append('forward')
            elif goal == 'north':
                actions.append('right')
            elif goal == 'south':
                actions.append('left')
        if current_loc == 'west':
            if goal == 'north':
                actions.append('left')
            elif goal == 'east':
                actions.append('forward')
            elif goal == 'south':
                actions.append('left')
        if current_loc == 'north':
            if goal == 'east':
                actions.append('left')
            elif goal == 'south':
                actions.append('straight')
            elif goal == 'west':
                actions.append('right')
        current_loc = tuple[1]
    return actions
    
def execute(self, inputs, outputs, gvm):
    n1 = Node("Aachen")
    n2 = Node("Eupen")
    n3 = Node("Vaals")
    n4 = Node("Kohlscheid")
    n5 = Node("Stolberg")
    n1.successor.append(Neighbour(n2, 2, 'south', 'north'))
    n1.successor.append(Neighbour(n3, 50, 'west', 'east'))
    n1.successor.append(Neighbour(n4, 2, 'north', 'south'))
    n1.successor.append(Neighbour(n5, 2, 'east', 'west'))

    n2.successor.append(Neighbour(n3, 3, 'west', 'south'))
    n2.successor.append(Neighbour(n1, 2, 'north', 'south'))
    n2.successor.append(Neighbour(n5, 3, 'east', 'south'))

    n3.successor.append(Neighbour(n4, 3, 'north', 'west'))
    n3.successor.append(Neighbour(n1, 2, 'east', 'west'))
    n3.successor.append(Neighbour(n2, 3, 'south', 'west'))

    n4.successor.append(Neighbour(n3, 3, 'west', 'north'))
    n4.successor.append(Neighbour(n1, 2, 'south', 'north'))
    n4.successor.append(Neighbour(n5, 3, 'east', 'north'))

    n5.successor.append(Neighbour(n4, 3, 'north', 'east'))
    n5.successor.append(Neighbour(n1, 2, 'west', 'east'))
    #n5.successor.append(Neighbour(n2, 3, 'south', 'east'))

    adjazenz = [n1,n2,n3,n4,n5]
    heuristic = dict()
    heuristic[n1] = 10.5
    heuristic[n2] = 3.5
    heuristic[n3] = 0
    heuristic[n4] = 30.5
    heuristic[n5] = 3
    '''heuristic[n1] = 1.5
    heuristic[n2] = 3.5
    heuristic[n3] = 0
    heuristic[n4] = 3.5
    heuristic[n5] = 3'''
    start = n5
    goal = n3
    pathfinding = AStar(heuristic,start,goal)
    path_backwards = []
    direction_sequenz = []
    goal = pathfinding.astar()
    while goal.predecessor[0] != 0:
        print (goal.name)
        if goal == start:
            break
        print("action: ", goal.predecessor[1].action, "loc: ", goal.predecessor[1].loc)
        #path_backwards.append(['west', 'east'])
        path_backwards.append([goal.predecessor[1].action, goal.predecessor[1].loc])
        goal = goal.predecessor[0]
    print("create PATH")
    for action in reversed(path_backwards):
        direction_sequenz.append(action)
    #for action in direction_sequenz:
        #print(action)
    direction_sequenz = action_sequenz(direction_sequenz, 'south')
    for action in direction_sequenz:
        print(action)
    action_queue = deque(direction_sequenz)
    gvm.set_variable("action_sequence", action_queue, per_reference=True)
    #self.preemptive_wait(2)
    return "success"
