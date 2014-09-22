# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
    Returns the start state for the search problem 
    """
    util.raiseNotDefined()
    
  def isGoalState(self, state):
    """
     state: Search state

    Returns True if and only if the state is a valid goal state
    """
    util.raiseNotDefined()

  def getSuccessors(self, state):
    """
     state: Search state

    For a given state, this should return a list of triples, 
    (successor, action, stepCost), where 'successor' is a 
    successor to the current state, 'action' is the action
    required to get there, and 'stepCost' is the incremental 
    cost of expanding to that successor
    """
    util.raiseNotDefined()

  def getCostOfActions(self, actions):
    """
    actions: A list of actions to take

    This method returns the total cost of a particular sequence of actions.  The sequence must
    be composed of legal moves
    """
    util.raiseNotDefined()

           
# CODE WORK STARTS HERE

# MAKE A NODE CLASS
# YOU MAY NOT FOLLOW THE SUGGESTED CODE BELO
class Node:
  """From AIMA: A node in a search tree. Contains a pointer 
    to the parent (the node that this is a successor of) 
    and to the actual state for this node. Note that if 
    a state is arrived at by two paths, then there are 
    two nodes with the same state.  Also includes the 
    action that got us to this state, and the total 
    path_cost (also known as g) to reach the node.  
      Other functions may add an f and h value."""

  def __init__(self, state, parent=None, action=[], path_cost = 0):
    "Create a search tree Node, derived from a parent by an action."
    self.state = state
    self.parent = parent
    self.action = action
    self.path_cost = path_cost
    pass
          
  def __repr__(self):
    return "<Node %s>" % (self.state,)

  def nodePath(self):
    "Create a list of nodes from the root to this node."
    pass
  
  def path(self):
    "Create a path of actions from the start to the current state"
    pass

  def expand(self, problem):
    "Return a list of nodes reachable from this node. [Fig. 3.8]"
    pass


# IMPLEMENT GRAPH SEARCH
# YOU MAY NOT FOLLOW THE SUGGESTED CODE BELOW
def graphSearch(problem, fringe):
  """Search through the successors of a problem to find a goal.
    The argument fringe should be an empty queue. [Fig. 3.18]"""
  startstate = problem.getStartState()
  fringe.push(Node(problem.getStartState()))
  pass


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]


# IMPLEMENT DEPTH-FIRST SEARCH
# YOU MAY NOT FOLLOW THE SUGGESTED CODE BELOW
def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  """

  fringe = util.Stack()
  expanded = set()
  fringe.push((problem.getStartState(),[],0))
  #print str(problem.getStartState)
  while not fringe.isEmpty():
    curState, curMoves, curCost = fringe.pop()
    #print curState, curMoves, curCost

    if(curState in expanded):
       continue

    expanded.add(curState)
    #print expanded
    if problem.isGoalState(curState):
      #print 'jed'
      return curMoves

    for state, direction, cost in problem.getSuccessors(curState):
      fringe.push((state, curMoves+[direction], curCost))
      #curState, curMoves, curCost = fringe.pop();
      #print curState, curMoves, curCost
  return []



  #return graphSearch(problem, util.Stack())

# IMPLEMENT BREADTH-FIRST SEARCH
def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  #util.raiseNotDefined()
  fringe = util.Queue()
  expanded = set()
  fringe.push((problem.getStartState(),[],0)) 
  #print str(problem.getStartState)
  while not fringe.isEmpty():
    curState, curMoves, curCost = fringe.pop()
    #print curState, curMoves, curCost

    if(curState in expanded):
      continue

    expanded.add(curState)
    #print expanded
    if problem.isGoalState(curState):
      #print 'jed'
      return curMoves

    for state, direction, cost in problem.getSuccessors(curState):
      fringe.push((state, curMoves+[direction], curCost))
      #curState, curMoves, curCost = fringe.pop();
      #print curState, curMoves, curCost
  return []

# IMPLEMENT UNIFORM-COST SEARCH      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  # #util.raiseNotDefined()

  #     #curState, curMoves, curCost = fringe.pop();
  #     #print curState, curMoves, curCost
  node = Node(state=problem.getStartState(),path_cost=0,action=[])    
  #fringe = util.PriorityQueue()
  # fringe.push(node,0)
  fringe = []
  fringe.insert(0,node)
  expanded = set()
  # #catchBasin = util.PriorityQueue()
  while fringe:
    fringe = sorted(fringe,key = lambda x: x.path_cost, reverse=True)
    node = fringe.pop()
    #print node 
    #stateList.append(node.state)
  #ut.sort(key=lambda x: x.count, reverse=True)
  #   node = fringe.pop()
    if problem.isGoalState(node.state):
      print node.action
      return node.action    #NOT SURE edit later
    if( node.state in expanded):
      continue
    expanded.add(node.state)
  #   for state, direction, cost in problem.getSuccessors(curState)
    for state, direction, cost in problem.getSuccessors(node.state):
      child = Node(parent=node,state=state,action=(node.action+[direction]),path_cost=cost) #NOTSURE baka cost + cost
      #print child.action
      isInsideFringe = False
      existingInstance = None
      for instance in fringe:
        if child.state == instance.state:
          existingInstance = instance
          #print existingInstance.path_cost, child.path_cost
          isInsideFringe = True
      if child.state not in expanded or isInsideFringe==False:
        fringe.append(child)
      elif isInsideFringe:
        #print existingInstance.path_cost, child.path_cost
        if existingInstance.path_cost > child.path_cost:
          existingInstance = child
      #print child.path_cost

  return []

  #       fringe.push(child,child.path_cost)
  #     elif child.state in stateList[]:
  #       # popped = fringe.pop(child.state)
  #       for





  raise KeyError  

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

# IMPLEMENT A* SEARCH      
def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  node = Node(state=problem.getStartState(),path_cost=0,action=[])    
  #fringe = util.PriorityQueue()
  # fringe.push(node,0)
  fringe = []
  fringe.insert(0,node)
  expanded = set()
  stateList = []
  # #catchBasin = util.PriorityQueue()
  while fringe:
    fringe = sorted(fringe,key = lambda x: x.path_cost, reverse=True)
    node = fringe.pop()
    #print node 
    #stateList.append(node.state)
  #ut.sort(key=lambda x: x.count, reverse=True)
  #   node = fringe.pop()
    if problem.isGoalState(node.state):
      #print node.action
      return node.action    #NOT SURE edit later
    if( node.state in expanded):
      continue
    expanded.add(node.state)
  #   for state, direction, cost in problem.getSuccessors(curState)
    for state, direction, cost in problem.getSuccessors(node.state):
      prior = heuristic(state,problem)
      child = Node(parent=node,state=state,action=(node.action+[direction]),path_cost=cost+prior) #NOTSURE baka cost + cost
      #print child.action
      isInsideFringe = False
      existingInstance = None
      for instance in fringe:
        if child.state == instance.state:
          existingInstance = instance
          isInsideFringe = True
      if child.state not in expanded or isInsideFringe==False:
        fringe.append(child)
      elif isInsideFringe:
        print existingInstance.path_cost, child.path_cost
        if existingInstance.path_cost > child.path_cost:
          existingInstance = child
      #print child.path_cost

  return []
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch