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
  queue.push((problem.getStartState(), []))
  explored = []
  while not queue.isEmpty():
    current, cameFrom = queue.pop()
    for pos, move, cost in problem.getSuccessors(current):
      if problem.isGoalState(pos):
        cameFrom.append(move)
        return cameFrom
      if not pos in explored:
        successorCameFrom = list(cameFrom)
        successorCameFrom.append(move)
        queue.push((pos, successorCameFrom))
        explored.append(pos)

  print "Unable to find any path to the goal!"
  return []
      
def uniformCostSearch(problem):
  queue = util.PriorityQueue()
  queue.push((problem.getStartState(), []), 0)
  explored = []
  while not queue.isEmpty():
    current, cameFrom = queue.pop()

    if problem.isGoalState(current):
      return cameFrom

    explored.append(current)

    for successor in problem.getSuccessors(current):
      if not successor[0] in explored:
        successorCameFrom = list(cameFrom)
        successorCameFrom.append(successor[1])
        queue.push((successor[0], successorCameFrom), problem.getCostOfActions(successorCameFrom))
  print "Unable to find any path to the goal!"
  return []

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def lowest(openSet, fScore):
  from sys import maxint
  lowestNode = None
  lowestScore = maxint
  for node in openSet:
    if fScore[node] < lowestScore:
      lowestNode = node
      lowestScore = fScore[node]

  return lowestNode


def aStarSearch(problem, heuristic=nullHeuristic):
  closedSet = []
  openSet = []
  openSet.append(problem.getStartState())
  cameFrom = {}
  gScore = {}
  gScore[problem.getStartState()] = 0
  fScore = {}
  fScore[problem.getStartState()] = heuristic(problem.getStartState(), problem)
  while len(openSet) > 0:
    current = lowest(openSet, fScore)
    if problem.isGoalState(current):
      return retracePath(cameFrom, current)
    openSet.remove(current)
    closedSet.append(current)
    for successor in problem.getSuccessors(current):
      neighbor = successor[0]
      if neighbor in closedSet:
        continue
      neighborScore = gScore[current] + 1
      if not (neighbor in openSet):
        openSet.append(neighbor)
      elif neighborScore >= gScore[neighbor]:
        continue

      cameFrom[neighbor] = (current, successor[1])
      gScore[neighbor] = neighborScore
      fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, problem)

  print "Unable to find solution!"
  return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
