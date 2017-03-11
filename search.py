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
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  stack = util.Stack()
  stack.push(problem.getStartState())
  children = {}
  visited = {}
  while not stack.isEmpty():
    current = stack.pop()
    if not current in visited:
      visited[current] = True
      for successor in problem.getSuccessors(current):
        if not successor[0] in visited:
          children[successor[0]] = (current, successor[1])
        stack.push(successor[0])
        if problem.isGoalState(successor[0]):
          return retracePath(children, successor[0])
  print "Unable to find any path to the goal!"
  return []

def retracePath(children, goal):
  path = []
  parent = goal
  while True:
    if parent in children:
      child = children[parent]
      path.append(child[1])
      parent = child[0]
    else:
      break
  path.reverse()
  return path

def breadthFirstSearch(problem):
  queue = util.Queue()
  queue.push(problem.getStartState())
  children = {}
  visited = {}
  while not queue.isEmpty():
    current = queue.pop()
    if not current in visited:
      visited[current] = True
      for successor in problem.getSuccessors(current):
        if not successor[0] in visited:
          children[successor[0]] = (current, successor[1])
        queue.push(successor[0])
        if problem.isGoalState(successor[0]):
          return retracePath(children, successor[0])
  print "Unable to find any path to the goal!"
  return []
      
def uniformCostSearch(problem):
  queue = util.PriorityQueue()
  queue.push(problem.getStartState(), 0)
  children = {}
  visited = {}
  while not queue.isEmpty():
    current = queue.pop()
    if not current in visited:
      visited[current] = True
      for successor in problem.getSuccessors(current):
        if not successor[0] in visited:
          children[successor[0]] = (current, successor[1])
        queue.push(successor[0], successor[2])
        if problem.isGoalState(successor[0]):
          return retracePath(children, successor[0])
  print "Unable to find any path to the goal!"
  return []

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
