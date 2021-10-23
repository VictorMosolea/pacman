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


import copy
import random

from game import Directions
from util import *


class Node:
    def __init__(self, state, parent, action, cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost


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
        raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def randomSearch(problem):
    """
    utils.Insert(Comments)
    """
    mystate = problem.getStartState()
    list = []
    while not problem.isGoalState(mystate):
        succesor = random.choice(problem.getSuccessors(mystate))
        list.append(succesor[1])
        mystate = succesor[0]
    return list


def depthFirstSearch(problem):
    """
    utils.Insert(Comments)
    """
    solution = []
    visited = []
    mystate = problem.getStartState()
    stack = Stack()
    curentNode = Node(mystate, None, None, 0)
    stack.push(curentNode)
    while(not problem.isGoalState(curentNode.state) and not stack.isEmpty()):
        curentNode = stack.pop()
        visited.append(curentNode.state)
        if (problem.isGoalState(curentNode.state)):
            lastNode = curentNode
            break

        succesors = problem.getSuccessors(curentNode.state)
        for succesor in succesors:
            succesorState = succesor[0]
            if succesorState not in visited:
                nextNode = Node(succesorState, curentNode,
                                succesor[1], curentNode.cost + 1)
                stack.push(nextNode)

    while lastNode.parent is not None:
        solution.append(lastNode.action)
        lastNode = lastNode.parent
    solution.reverse()
    return solution


def breadthFirstSearch(problem):
    """
    utils.Insert(Comments)
    """
    solution = []
    visited = []
    mystate = problem.getStartState()
    stack = Queue()
    visited.append(mystate)
    curentNode = Node(mystate, None, None, 0)
    stack.push(curentNode)
    while(not problem.isGoalState(curentNode.state) and not stack.isEmpty()):
        curentNode = stack.pop()
        if (problem.isGoalState(curentNode.state)):
            lastNode = curentNode
            break
        succesors = problem.getSuccessors(curentNode.state)
        for succesor in succesors:
            succesorState = succesor[0]
            if succesorState not in visited:
                visited.append(succesorState)
                nextNode = Node(succesorState, curentNode,
                                succesor[1], curentNode.cost + 1)
                stack.push(nextNode)

    while lastNode.parent is not None:
        solution.append(lastNode.action)
        lastNode = lastNode.parent
    solution.reverse()
    return solution


def uniformCostSearch(problem):
    """
    utils.Insert(Comments)
    """
    solution = []
    mystate = problem.getStartState()
    store = PriorityQueue()
    map = {}
    curentNode = Node(mystate, None, None, 0)
    store.push(curentNode, curentNode.cost)
    map[mystate] = (curentNode)
    lastNode = curentNode
    while (not problem.isGoalState(curentNode.state) and not store.isEmpty()):
        curentNode = store.pop()
        if (problem.isGoalState(curentNode.state)):
            lastNode = curentNode
            break
        succesors = problem.getSuccessors(curentNode.state)
        for (succesorState, succesorAction, succesorCost) in succesors:
            nextNode = Node(succesorState, curentNode,
                            succesorAction, curentNode.cost + succesorCost)
            if (not map.has_key(succesorState)):
                map[nextNode.state] = nextNode
                store.push(nextNode, nextNode.cost)
            elif nextNode.cost < map[succesorState].cost:
                store.update(nextNode, nextNode.cost)
                map[nextNode.state] = nextNode

    while lastNode.parent is not None:
        solution.append(lastNode.action)
        lastNode = lastNode.parent
    solution.reverse()
    return solution


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
  solution = []
  mystate = problem.getStartState()
  store = PriorityQueue()
  map = {}
  curentNode = Node(mystate, None, None, 0)
  store.push(curentNode, curentNode.cost)
  map[mystate] = (curentNode)
  lastNode = curentNode
  while (not problem.isGoalState(curentNode.state) and not store.isEmpty()):
    curentNode = store.pop()
    if (problem.isGoalState(curentNode.state)):
      lastNode = curentNode
      break
    succesors = problem.getSuccessors(curentNode.state)
    for (succesorState, succesorAction, succesorCost) in succesors:
      nextNode = Node(succesorState, curentNode,
                      succesorAction, curentNode.cost + succesorCost)
      if (not map.has_key(succesorState)):
          map[nextNode.state] = nextNode
          store.push(nextNode, nextNode.cost + heuristic(succesorState,problem))
      elif nextNode.cost < map[succesorState].cost:
          store.push(nextNode, nextNode.cost + heuristic(succesorState,problem))
          map[nextNode.state] = nextNode

  while lastNode.parent is not None:
    solution.append(lastNode.action)
    lastNode = lastNode.parent
  solution.reverse()
  return solution


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
