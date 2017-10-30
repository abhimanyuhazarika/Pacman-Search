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

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"


    #creating a stack to store the frontier nodes in LIFO order
    frontier = util.Stack()
    #dict for storing traversed nodes
    traversed = {}
    #getting the initial state
    state = problem.getStartState()
    #storing the parent node, action performed to reach the node, current state of the node in a dict
    node = {"parent": None,"action":None,"state":state }
    
    frontier.push(node)
    i = 0;

    while not frontier.isEmpty():
        
      #get the first node and check the successors in LIFO order untill the goal is reached
      node = frontier.pop()
      state = node["state"]
      if state in traversed.values():         
        continue
      traversed[i] = state
      i=i+1
      #check if the state found is the goal state
      if problem.isGoalState(state) == True:
       break
      for successor in problem.getSuccessors(state):
        if not successor[0] in traversed.values():
          frontier_node= {"parent":node, "action":successor[1], "state":successor[0]}
          frontier.push(frontier_node)
    
    #put the actions in a FIFO order using a Queue for the pacman to reach the goal
    actions = util.Queue()
    while node["action"] != None:
      actions.push(node["action"])
      node = node["parent"]
      
    return actions.list

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    #creating a Queue to store the frontier nodes in FIFO order
    frontier = util.Queue()
    #dict for storing traversed nodes
    traversed = {}
    #getting the initial state
    state = problem.getStartState()
    #storing the parent node, action performed to reach the node, current state of the node in a dict
    node = {"parent": None,"action":None,"state":state }
    
    frontier.push(node)
    
    #counter for the traversed dict
    i = 0;

    while not frontier.isEmpty():
        
      #get the first node and check the successors in FIFO order untill the goal is reached
      node = frontier.pop()
      state = node["state"]
      if state in traversed.values():         
        continue
      traversed[i] = state
      i=i+1
      #check if the state found is the goal state
      if problem.isGoalState(state) == True:
       break
      for successor in problem.getSuccessors(state):
        if not successor[0] in traversed.values():
          frontier_node= {"parent":node, "action":successor[1], "state":successor[0]}
          frontier.push(frontier_node)
    
    #put the actions in a FIFO order using a Queue for the pacman to reach the goal
    actions = util.Queue()
    while node["action"] != None:
      actions.push(node["action"])
      node = node["parent"]
      
    return actions.list
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    #creating a Priority Queue to store the frontier nodes on the order of cost value
    frontier = util.PriorityQueue()
    #dict for storing traversed nodes
    traversed = {}
    #getting the initial state
    state = problem.getStartState()
    #storing the parent node, action performed to reach the node, current state of the node, cost of the node in a dict
    node = {"parent": None,"action":None,"state":state,"cost":0 }
    
    #putting cost for initial state as 0
    frontier.push(node,0)
    i = 0;

    while not frontier.isEmpty():
        
      #get the first node and get the successors in order of cost untill the goal is reached with minimum cost
      node = frontier.pop()
      state = node["state"]
      cost = node["cost"] 
      if state in traversed.values():         
        continue
      traversed[i] = state
      i=i+1
      #check if the state found is the goal state
      if problem.isGoalState(state) == True:
       break
      #calculate the cost to reach the node and put it in the queue for priority
      for successor in problem.getSuccessors(state):
        if not successor[0] in traversed.values():
          frontier_node= {"parent":node, "action":successor[1], "state":successor[0],"cost":successor[2]+cost}
          frontier.push(frontier_node,successor[2]+cost)
    
    #put the actions in a FIFO order using a Queue for the pacman to reach the goal
    actions = util.Queue()
    while node["action"] != None:
      actions.push(node["action"])
      node = node["parent"]
      
    return actions.list

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Ssearch the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    
    #creating a Priority Queue to store the frontier nodes on the order of cost value
    frontier = util.PriorityQueue()
    #getting the initial state
    state = problem.getStartState()
    #storing the parent node, action performed to reach the node, current state of the node, cost of the node and heuristic value in a dict
    node = {"state":state, "parent" : None, "action":None, "cost":0, "heuristic":heuristic(state, problem)}
    #setting cost for initial state as 0 and added cost with heuristic and inserted it in the priority queue
    frontier.push(node, node["cost"] + node["heuristic"])
    i=0
    #dict for storing traversed nodes
    traversed = {}
    actions = []
    
    while not frontier.isEmpty():
        #get the first node and get the successors in order of cost+heuristic untill the goal is reached with minimum cost+heuristic
        node = frontier.pop()
        state = node["state"]
        cost = node["cost"]
        #check if the node is already traversed
        if state in traversed.values():         
            continue
        traversed[i] = state
        i+=1
        #check if the state found is the goal state
        if problem.isGoalState(state) == True:
            break
        #calculate the cost+heuristic to reach the node and put it in the queue for priority calculation
        for successor in problem.getSuccessors(state):
            if not successor[0] in traversed.values():
                frontier_node = {"parent": node, "state" : successor[0], "action" : successor[1], "cost" : successor[2] + cost}
                frontier_node["heuristic"] = heuristic(frontier_node["state"], problem)
                frontier.push(frontier_node, frontier_node["cost"] + frontier_node["heuristic"])
   
    #put the actions in a FIFO order using a Queue for the pacman to reach the goal
    actions = util.Queue()
    while node["action"] != None:
        actions.push(node["action"])
        node = node["parent"]
        #print actions
        
    return actions.list

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
