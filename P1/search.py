# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


class Node:
    
    def __init__(self, state, path, cost=1):
        self.state = state
        self.path = path
        self.cost = cost
    
    def getState(self):
        return self.state
    
    def getPath(self):
        return self.path

    def getCost(self):
        return self.cost
        
    def __eq__(self, other):
        if type(other) is type(self):
            return self.getState() == other.getState()

    def __ne__(self, other):
        return not self.__eq__(self, other)


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """    
    frontier = util.Stack()
    frontier.push(makeNode(problem.getStartState()))
    closed = []
    
    return recursiveDFS(problem, frontier, closed)

def recursiveDFS(problem, frontier, closed):
    node = frontier.pop()
    
    if(problem.isGoalState((node.getState()))):
        return node.getPath()
    
    for suc in problem.getSuccessors(node.getState()):
        if suc[0] not in closed:
            newNode = makeNode(suc[0], node, suc)
            frontier.push(newNode)
            closed.append(node.getState())
    
    return recursiveDFS(problem, frontier, closed)

def makeNode(state, node=None, successor=None):
    if node and successor:
        arr = list(node.getPath())
        arr.append(successor[1])
        return Node(state, arr, node.getCost() + successor[2])
    else:
        return Node(state, [])

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    frontier = util.Queue()
    frontier.push(makeNode(problem.getStartState()))
    closed = []
    
    while not frontier.isEmpty():
        node = frontier.pop()
        closed.append(node)

        if problem.isGoalState(node.getState()):
            return node.getPath()

        for successor in problem.getSuccessors(node.getState()):
            newnode = makeNode(successor[0], node, successor)
            if newnode not in closed and not listContains(frontier.list, newnode):
                frontier.push(newnode)

    print "Deu ruim no breadth first search"
    return []

def listContains(alist, node):
    for n in alist:
        if n == node:
            return True
    return False

def heapContains(alist, node):
    for n in alist:
        if n[2] == node:
            return True
    return False

def heapContainsHigherCost(alist, node):
    for n in alist:
        if n[2] == node and n[0] > node.getCost():
            return True

    return False


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()
    frontier.push(makeNode(problem.getStartState()), 0)
    closed = []
    
    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node.getState()):
            return node.getPath()

        closed.append(node.getState())
        for successor in problem.getSuccessors(node.getState()):
            newnode = makeNode(successor[0], node, successor)
            
            if newnode.getState() not in closed and not heapContains(frontier.heap, newnode):
                frontier.push(newnode, newnode.getCost())
            else:
                if heapContainsHigherCost(frontier.heap, newnode):
                    frontier.update(newnode, newnode.getCost())

    print "Deu ruim no uniform cost search"
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
   
    print "Deu ruim no a* search"
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
