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
from dataclasses import dataclass, field


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self) -> tuple:
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()
        return 0, 0  # Appended to get rid of warnings

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state) -> list:
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()
        return []  # Appended to get rid of warnings

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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem: SearchProblem):
    """
    Finds the route to a goal state of the problem using the depth first tree search algorithm, with loop detection.
    Returns a list of actions (e.g. "North", "West", "South", "East")

    :param problem: The search problem
    :return:        List of actions that lead from the start state of the problem to a goal state
    """

    # Applying the general tree search algorithm, with loop detection:
    frontier = util.Stack()
    frontier.push(Node(tuple(), problem.getStartState()))
    reached = set()

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node.game_state):
            return list(node.actions)

        if node.game_state not in reached:
            reached.add(node.game_state)
            for successor_game_state, successor_action, _ in problem.getSuccessors(node.game_state):
                new_node_actions = node.actions + (successor_action,)  # Creates a tuple with one extra element
                new_node = Node(new_node_actions, successor_game_state)
                frontier.push(new_node)

    return None


def breadthFirstSearch(problem: SearchProblem):
    """
    Finds the route to a goal state of the problem using the breadth first tree search algorithm, with loop detection.
    Returns a list of actions (e.g. "North", "West", "South", "East")

    :param problem: The search problem
    :return:        List of actions that lead from the start state of the problem to a goal state
    """

    # Since I've only got a weekend for this project, rather than writing a new function which generalises these
    # functions, simply copying previously written code and slightly altering it will have to do.

    # Applying the general tree search algorithm, with loop detection:
    frontier = util.Queue()
    frontier.push(Node(tuple(), problem.getStartState()))
    reached = set()

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node.game_state):
            return list(node.actions)

        if node.game_state not in reached:
            reached.add(node.game_state)
            for successor_game_state, successor_action, _ in problem.getSuccessors(node.game_state):
                new_node_actions = node.actions + (successor_action,)  # Creates a tuple with one extra element
                new_node = Node(new_node_actions, successor_game_state)
                frontier.push(new_node)

    return None


def uniformCostSearch(problem: SearchProblem):
    """
    Finds the route to a goal state of the problem using the uniform cost tree search algorithm, with loop detection.
    Returns a list of actions (e.g. "North", "West", "South", "East")

    :param problem: The search problem
    :return:        List of actions that lead from the start state of the problem to a goal state
    """

    # Once again, simply copying and adjusting previous code.
    # Applying the general tree search algorithm, with loop detection:
    frontier = util.PriorityQueue()
    frontier.push(Node(tuple(), problem.getStartState()), 0)
    reached = set()

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node.game_state):
            return list(node.actions)

        if node.game_state not in reached:
            reached.add(node.game_state)
            for successor_game_state, successor_action, successor_cost in problem.getSuccessors(node.game_state):
                new_node_actions = node.actions + (successor_action,)  # Creates a tuple with one extra element
                new_node_cost = node.cost + successor_cost

                new_node = Node(new_node_actions, successor_game_state, new_node_cost)
                frontier.update(new_node, new_node_cost)

    return None


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """
    Finds the route to a goal state of the problem using the A* tree search algorithm, with loop detection. Assumes the
    given heuristic is consistent.
    Returns a list of actions (e.g. "North", "West", "South", "East")

    :param problem:     The search problem
    :param heuristic:   The heuristic used in the A* algorithm to represent "distance from goal". MUST be consistent.
    :return:            List of actions that lead from the start state of the problem to a goal state
    """

    # Once again, simply copying and adjusting previous code.
    # Applying the general tree search algorithm, with loop detection:
    frontier = util.PriorityQueue()
    frontier.push(Node(tuple(), problem.getStartState()), 0)
    reached = set()

    while not frontier.isEmpty():
        node = frontier.pop()

        if problem.isGoalState(node.game_state):
            return list(node.actions)

        if node.game_state not in reached:
            reached.add(node.game_state)
            for successor_game_state, successor_action, successor_cost in problem.getSuccessors(node.game_state):
                new_node_actions = node.actions + (successor_action,)  # Creates a tuple with one extra element
                new_node_cost = node.cost + successor_cost

                new_node_priority = new_node_cost + heuristic(successor_game_state, problem)

                new_node = Node(new_node_actions, successor_game_state, new_node_cost)
                frontier.update(new_node, new_node_priority)  # Note: node priority is different from node cost

    return None


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch


###################################
# Self-defined classes/algorithms #
###################################

@dataclass
class Node:
    """
    Node represents a search node in a graph search for pac-man. A node stores the game_state represented by two integer
    coordinates, the actions needed to get to said game_state, as well as the cost to move to the game_state from start.

    Two nodes are considered the same if their locations are the same.

    Note that we essentially abuse a property from PriorityQueue in util, where if we have two nodes with the same
    game_state, the one with the lowest cost (and thus lowest priority) will stay in the PriorityQueue. This means that
    the actions saved in the node for that game_state, are the actions with the lowest cost to get there.
    """

    actions: tuple = field(default=tuple(), compare=False)
    game_state: object = field(default=None)
    cost: int = field(default=0, compare=False)
