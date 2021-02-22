import sys
import numpy as np

LENGTH = 3

class Node():
    """Node"""
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

class StackFrontier():
    def __init__(self):
        self.frontier = []
    
    def add(self, node):
        self.frontier.append(node)

    def empty(self):
        return len(self.frontier) == 0

    def contains_state(self, state):
        for element in self.frontier:
            if (state == element.state).all():
                return True
        return False

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            # get the last node and split the array
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node
    def __str__(self):
        return f'a stack datastructure for dfs algortihm'

class QueueFrontier(StackFrontier):
    """queue frontier"""
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            # get the last node and split the array
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node
        
        

class Puzzle8():
    def __init__(self, method, puzzle):
        values = puzzle.split(',')
        # only if the array is equal to nine continue
        if len(values) != 9:
            raise Exception(f'the input array must be 9 length and was enter a {len(values)} array length')

        for i, value in enumerate(values):
            values[i] = int(value)

        self.startState = np.reshape(values, (3,3))

        # the goal always is the same.
        self.goal = np.array(
        [ 
            [0, 1, 2], 
            [3, 4, 5], 
            [6, 7, 8] 
        ])

        self.solution = None
        self.method = method

    def neighbors(self, state):
        # first search the agent.
        agent = np.where(state == 0)
        y, x = agent[0][0], agent[1][0]
        actions = [
            ('up', (y - 1, x)),
            ('down', (y + 1, x)),
            ('right', (y, x + 1)),
            ('left', (y , x -1)),
        ]
        
        result = []
        for action, (newy, newx) in actions:
            if 0 <= newy < LENGTH and 0 <= newx < LENGTH:
                #copy the state and then swap the elements
                newState = np.copy(state)
                newState[y][x], newState[newy][newx] = newState[newy][newx], newState[y][x]
                result.append((action, newState))

        return result


    def solve(self):
        """Finds a solution to maze, if one exists."""
        # Keep track of number of states explored
        self.num_explored = 0

        #Start with the frontier that contains the initial state
        start = Node(state=self.startState, parent=None, action=None)
        #select the method
        if self.method == 'dfs':
            frontier = StackFrontier()
        else:
            frontier = QueueFrontier()
        frontier.add(start)

        #Start with an empty explored set
        self.explored = set()
        
        #repeat until reach the goal
        while True:
            #if the frontier is empty, then no solution.
            if frontier.empty():
                raise Exception('no solution')
            #remove a node from the frontier.
            node = frontier.remove()
            self.num_explored += 1
            #if node contains goal state, return the solution.
            if (node.state == self.goal).all():
                actions = []
                while node.parent is not None:
                    actions.append(node.action)
                    node = node.parent
                actions.reverse()
                self.solution = actions
                return
            #Add the node to the explored set
            #it's no possible put a list in the set data structure
            #for that reason we put a tuple. 
            self.explored.add(tuple(list(node.state.reshape(-1))))
            #expand node, add resulting nodes to the frontier if they aren't already in the frontier or the explored set.
            for action, state in self.neighbors(node.state):
                fixState = tuple(list(state.reshape(-1)))
                if not frontier.contains_state(state) and fixState not in self.explored:
                    child = Node(state = state, parent = node, action = action)
                    frontier.add(child)

    def print(self):
        print(self.startState)
        print(self.solution)
        print(f'node explored: {self.num_explored}')
        print(f'path cost: {len(self.solution)}')

puzzle = Puzzle8(sys.argv[1], sys.argv[2])
print("Solving...")
puzzle.solve()
puzzle.print()
