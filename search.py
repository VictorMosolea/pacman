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
    def __init__(self, pos, parent, action, cost):
        self.pos = pos
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
    current_pos = problem.getStartState()
    actions = []
    while not problem.isGoalState(current_pos):
        next_state = random.choice(problem.getSuccessors(current_pos))
        actions.append(next_state[1])
        current_pos = next_state[0]
    return actions


def depthFirstSearch(problem):
    actions = []
    visited = []
    current_pos = problem.getStartState()
    stack = Stack()
    current_node = Node(current_pos, None, None, 0)
    stack.push(current_node)
    while(not problem.isGoalState(current_node.pos) and not stack.isEmpty()):
        current_node = stack.pop()
        visited.append(current_node.pos)
        if (problem.isGoalState(current_node.pos)):
            last_node = current_node
            break

        nexts = problem.getSuccessors(current_node.pos)
        for n in nexts:
            next_pos = n[0]
            if next_pos not in visited:
                next_node = Node(next_pos, current_node,
                                n[1], current_node.cost + 1)
                stack.push(next_node)
    if last_node == None:
        return []
    while last_node.parent is not None:
        actions.append(last_node.action)
        last_node = last_node.parent
    return actions[::-1]


def breadthFirstSearch(problem):
    actions = []
    visited = []
    last_node = None
    current_pos = problem.getStartState()
    queue = Queue()
    visited.append(current_pos)
    current_node = Node(current_pos, None, None, 0)
    queue.push(current_node)
    while(not problem.isGoalState(current_node.pos) and not queue.isEmpty()):
        current_node = queue.pop()
        if (problem.isGoalState(current_node.pos)):
            last_node = current_node
            break
        nexts = problem.getSuccessors(current_node.pos)
        for n in nexts:
            next_pos = n[0]
            if next_pos not in visited:
                visited.append(next_pos)
                next_node = Node(next_pos, current_node,
                                n[1], current_node.cost + 1)
                queue.push(next_node)
    if last_node == None:
        return []
    while last_node.parent is not None:
        actions.append(last_node.action)
        last_node = last_node.parent
    return actions[::-1]


def uniformCostSearch(problem):
    actions = []
    current_pos = problem.getStartState()
    queue = PriorityQueue()
    visited = {}
    current_node = Node(current_pos, None, None, 0)
    queue.push(current_node, current_node.cost)
    visited[current_pos] = (current_node)
    last_node = current_node
    while (not problem.isGoalState(current_node.pos) and not queue.isEmpty()):
        current_node = queue.pop()
        if (problem.isGoalState(current_node.pos)):
            last_node = current_node
            break
        nexts = problem.getSuccessors(current_node.pos)
        for (next_pos, next_action, next_cost) in nexts:
            next_node = Node(next_pos, current_node,
                            next_action, current_node.cost + next_cost)
            if (not visited.has_key(next_pos)):
                visited[next_node.pos] = next_node
                queue.push(next_node, next_node.cost)
            elif next_node.cost < visited[next_pos].cost:
                queue.update(next_node, next_node.cost)
                visited[next_node.pos] = next_node
    if last_node == None:
        return []
    while last_node.parent is not None:
        actions.append(last_node.action)
        last_node = last_node.parent
    return actions[::-1]


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    actions = []
    current_pos = problem.getStartState()
    queue = PriorityQueue()
    visited = {}
    current_node = Node(current_pos, None, None, 0)
    queue.push(current_node, current_node.cost)
    visited[current_pos] = (current_node)
    last_node = current_node
    while (not problem.isGoalState(current_node.pos) and not queue.isEmpty()):
        current_node = queue.pop()
        if (problem.isGoalState(current_node.pos)):
            last_node = current_node
            break
        nexts = problem.getSuccessors(current_node.pos)
        for (next_pos, next_action, next_cost) in nexts:
            next_node = Node(next_pos, current_node,
                            next_action, current_node.cost + next_cost)
            if (not visited.has_key(next_pos)):
                visited[next_node.pos] = next_node
                queue.push(next_node, next_node.cost + heuristic(next_pos,problem))
            elif next_node.cost < visited[next_pos].cost:
                queue.push(next_node, next_node.cost + heuristic(next_pos,problem))
                visited[next_node.pos] = next_node
    if last_node == None:
            return []
    while last_node.parent is not None:
        actions.append(last_node.action)
        last_node = last_node.parent
    return actions[::-1]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
