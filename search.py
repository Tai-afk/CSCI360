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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    startNode = problem.getStartState()

    #check if starting node is goal node, if it is return empty list
    if(problem.isGoalState(startNode)):
        return []

    q = util.Stack()
    visited = []

    #Pair consists of the node itself and the path it took to the node
    pathToNode = (startNode, [])

    q.push(pathToNode)
    while not q.isEmpty():
        pathToNode = q.pop()
        node = pathToNode[0]
        path = pathToNode[1]

        if(problem.isGoalState(node)):
            return path
        #if already visited, then continue, else add node to visited
        if node in visited:
            continue
        else:
            visited.append(node)

        #For each sucessor nodes, add direction it took to get to and the parameter as well
        for x, y, z in problem.getSuccessors(node):
            #create new path
            newPath = path + [y]
            #pair path and node together into the q
            newPair = (x, newPath)
            q.push(newPair)
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    #breadth first search is the same thing but uses queue instead of stack
    startNode = problem.getStartState()

    #check if starting node is goal node, if it is return empty list
    if(problem.isGoalState(startNode)):
        return []

    q = util.Queue()
    visited = []

    #Pair consists of the node itself and the path it took to the node
    pathToNode = (startNode, [])

    q.push(pathToNode)
    while not q.isEmpty():
        pathToNode = q.pop()
        node = pathToNode[0]
        path = pathToNode[1]

        if(problem.isGoalState(node)):
            return path
        #if already visited, then continue, else add node to visited
        if node in visited:
            continue
        else:
            visited.append(node)

        #For each sucessor nodes, add direction it took to get to and the parameter as well
        for x, y, z in problem.getSuccessors(node):
            #create new path
            newPath = path + [y]
            #pair path and node together into the q
            newPair = (x, newPath)
            q.push(newPair)
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    startNode = problem.getStartState()

    #check if starting node is goal node, if it is return empty list
    if(problem.isGoalState(startNode)):
        return []

    q = util.Queue()
    visited = []

    #Pair consists of the node itself and the path it took to the node,  and its  cost
    pathToNode = (startNode, [], 0)

    q.push(pathToNode, 0)
    while not q.isEmpty():
        pathToNode = q.pop()
        node = pathToNode[0][0]
        path = pathToNode[0][1]
        cost = pathToNode[1]
        if(problem.isGoalState(node)):
            return path
        #if already visited, then continue, else add node to visited
        if node in visited:
            continue
        else:
            visited.append(node)

        #For each sucessor nodes, add direction it took to get to and the parameter as well
        #x is the node add
        #y is the path
        #z is the cost
        for x, y, z in problem.getSuccessors(node):
            #create new path
            newPath = path + [y]
            #cost to get there
            val = cost + z
            #pair path and node together into the q
            newPair = (x, newPath,val)
            q.push(newPair, val)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """

    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    #Heuristic take the manhattan distance?
    startingNode = problem.getStartState()
    q = util.PriorityQueue()
    visited = []
    directions = []
    #get the heuristic from the starting node to the problem
    q.push((startingNode, directions), heuristic(startingNode, problem))

    while not q.isEmpty():
        pathToNode = q.pop()
        node = pathToNode[0][0]
        path = pathToNode[0][1]
        h_cost = pathToNode[1]

        if not node in visited:
            visited.append(node)

            if(problem.isGoalState(node)):
                return directions

            for new_node, dir, cost in problem.getSuccessors(node):
                #path
                new_path = path + [dir]
                #cost
                g_cost = h_cost + cost
                #input into the queue by the lowest heuristic cost + g cost
                q.push((new_node, new_path), heuristic(new_node, problem) + g_cost)
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
