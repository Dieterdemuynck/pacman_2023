# searchAgents.py
# ---------------
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
This file contains all of the agents that can be selected to control Pacman.  To
select an agent, use the '-p' option when running pacman.py.  Arguments can be
passed to your agent using '-a'.  For example, to load a SearchAgent that uses
depth first search (dfs), run the following command:

> python pacman.py -p SearchAgent -a fn=depthFirstSearch

Commands to invoke other search strategies can be found in the project
description.

Please only change the parts of the file you are asked to.  Look for the lines
that say

"*** YOUR CODE HERE ***"

The parts you fill in start about 3/4 of the way down.  Follow the project
description for details.

Good luck and happy searching!
"""

from typing import Tuple
from game import Directions
from game import Agent
from game import Actions
from game import Grid
import util
import time
import search
import pacman
from dataclasses import dataclass
from math import sqrt


class GoWestAgent(Agent):
    """An agent that goes West until it can't."""

    def getAction(self, state):
        """The agent receives a GameState (defined in pacman.py)."""
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


#######################################################
# This portion is written for you, but will only work #
#       after you fill in parts of search.py          #
#######################################################

class SearchAgent(Agent):
    """
    This very general search agent finds a path using a supplied search
    algorithm for a supplied search problem, then returns actions to follow that
    path.

    As a default, this agent runs DFS on a PositionSearchProblem to find
    location (1,1)

    Options for fn include:
      depthFirstSearch or dfs
      breadthFirstSearch or bfs


    Note: You should NOT change any code in SearchAgent
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):
        # Warning: some advanced Python magic is employed below to find the right functions and problems

        # Get the search function from the name and heuristic
        if fn not in dir(search):
            raise AttributeError(fn + ' is not a search function in search.py.')
        func = getattr(search, fn)
        if 'heuristic' not in func.__code__.co_varnames:
            print('[SearchAgent] using function ' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError(heuristic + ' is not a function in searchAgents.py or search.py.')
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))
            # Note: this bit of Python trickery combines the search algorithm and the heuristic
            self.searchFunction = lambda x: func(x, heuristic=heur)

        # Get the search problem type from the name
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError(prob + ' is not a search problem type in SearchAgents.py.')
        self.searchType = globals()[prob]
        print('[SearchAgent] using problem type ' + prob)

    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        if self.searchFunction == None: raise Exception("No search function provided for SearchAgent")
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions  = self.searchFunction(problem)  # Find a path
        if self.actions == None:
            self.actions = []
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)

    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class PositionSearchProblem(search.SearchProblem):
    """
    A search problem defines the state space, start state, goal test, successor
    function and cost function.  This search problem can be used to find paths
    to a particular point on the pacman board.

    The state space consists of (x,y) positions in a pacman game.

    Note: this search problem is fully specified; you should NOT change it.
    """

    def __init__(self, gameState, costFn = lambda x: 1, goal=(1, 1), start=None, warn=True, visualize=True):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print('Warning: this does not look like a regular search maze')

        # For display purposes
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # For display purposes only
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display):  # @UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist)  # @UndefinedVariable

        return isGoal

    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """

        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x,y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append( ( nextState, action, cost) )

        # Bookkeeping for display purposes
        self._expanded += 1  # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions. If those actions
        include an illegal move, return 999999.
        """
        if actions == None: return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x,y))
        return cost


class StayEastSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the West side of the board.

    The cost function for stepping into a position (x,y) is 1/2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)


class StayWestSearchAgent(SearchAgent):
    """
    An agent for position search with a cost function that penalizes being in
    positions on the East side of the board.

    The cost function for stepping into a position (x,y) is 2^x.
    """
    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)


def manhattanHeuristic(position, problem, info={}):
    """The Manhattan distance heuristic for a PositionSearchProblem"""
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):  # Are these... mutable default values? Huh? They're unused too?
    """The Euclidean distance heuristic for a PositionSearchProblem"""
    xy1 = position
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5


#####################################################
# This portion is incomplete.  Time to write code!  #
#####################################################

class CornersProblem(search.SearchProblem):
    """
    This search problem finds paths through all four corners of a layout.

    You must select a suitable state space and successor function
    """

    def __init__(self, startingGameState: pacman.GameState):
        """
        Stores the walls, pacman's starting position and corners.
        """
        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height-2, self.walls.width-2
        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print('Warning: no food in corner ' + str(corner))
        self._expanded = 0  # DO NOT CHANGE; Number of search nodes expanded

        # Self-defined additions to CornersProblem.__init__:
        print("Error 418: I'm a teapot")  # Oh hey, you spotted my Easter egg!

    def getStartState(self):
        """
        Returns the start state (in your state space, not the full Pacman state
        space)
        """
        return self.CornerProblemGameState(self.startingPosition, frozenset())

    def isGoalState(self, state: "CornerProblemGameState"):
        """
        Returns whether this search state is a goal state of the problem.
        """
        return state.corners_hit == set(self.corners)

    def getSuccessors(self, state: "CornerProblemGameState"):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
            For a given state, this should return a list of triples, (successor,
            action, stepCost), where 'successor' is a successor to the current
            state, 'action' is the action required to get there, and 'stepCost'
            is the incremental cost of expanding to that successor
        """

        successors = []

        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            # Add a successor state to the successor list if the action is legal
            x, y = state.location
            dx, dy = Actions.directionToVector(action)
            next_x, next_y = int(x + dx), int(y + dy)
            hits_wall = self.walls[next_x][next_y]

            if not hits_wall:
                successor_location = next_x, next_y
                successor_data = self._create_new_successor(state=state, successor_location=successor_location,
                                                            action=action)
                successors.append(successor_data)

        self._expanded += 1  # DO NOT CHANGE
        # Change is necessary to improve as a person... But okay, for now I won't, have a good day =)
        return successors

    def _create_new_successor(self, *, state, successor_location, action, step_cost=1):
        successor_corners_hit = state.corners_hit
        if successor_location in self.corners:
            # appends corner to successor's corners hit.
            successor_corners_hit = successor_corners_hit.union({successor_location})

        successor_state = self.CornerProblemGameState(successor_location, successor_corners_hit)
        return successor_state, action, step_cost

    def getCostOfActions(self, actions):
        """
        Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999.  This is implemented for you.
        """
        if actions is None:
            return 999999
        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999

        return len(actions)

    @dataclass(frozen=True, eq=True)
    class CornerProblemGameState:
        """
        Game state for the Corner Problem. This contains not only the location, but also which corners have already been
        hit.
        """
        location: tuple
        corners_hit: frozenset


def cornersHeuristic(state: CornersProblem.CornerProblemGameState, problem: CornersProblem):
    """
    A heuristic for the CornersProblem that you defined.

      state:   The current search state
               (a data structure you chose in your search problem)

      problem: The CornersProblem instance for this layout.

    This function should always return a number that is a lower bound on the
    shortest path from the state to a goal of the problem; i.e.  it should be
    admissible (as well as consistent).
    """
    corners = set(problem.corners)  # These are the corner coordinates
    walls = problem.walls  # These are the walls of the maze, as a Grid (game.py)

    # My approach:
    # Find the smallest manhattan distance needed to travel to reach each corner.
    height, width = problem.walls.height-2, problem.walls.width-2
    corners_to_hit = set(corners).difference(state.corners_hit)

    return _corner_distance(state.location, corners_to_hit, height=height, width=width)


def _corner_distance(current_location: tuple, corners_to_hit, height, width):
    """
    Calculates the minimum manhattan distance to travel to reach each corner in corners_to_hit, from current_location.
    This isn't my proudest achievement of a function, but it works.

    :param current_location: A tuple (x, y) representing the coordinates of pac-man's current location
    :param corners_to_hit:   An iterable of tuples representing the coordinates of the corners yet to be hit. These
                             corners MUST make a rectangle.
    :return:     The minimum manhattan distance to reach each corner yet to be hit.
    """
    if len(corners_to_hit) == 0:
        return 0

    corners = list(corners_to_hit)
    distance_pacman_to_corner = (util.manhattanDistance(corner, current_location)
                                 for corner in corners)
    short_side_length = min(height, width)
    long_side_length = max(height, width)

    if len(corners) == 4:
        # Reminder: distance to travel is one less than the length, e.g. from 1 to 5 requires 4 steps, but is 5 long.
        return min(distance_pacman_to_corner) + 2*(short_side_length-1) + long_side_length - 1
    if len(corners) == 3:
        # "Inner" corner has no effect, remove this from the list and find closest "outer" corner instead
        if corners[0][0] != corners[1][0] and corners[0][1] != corners[1][1]:
            # corners 0 and 1 are outer corners
            corners.pop(2)
        elif corners[0][0] != corners[2][0] and corners[0][1] != corners[2][1]:
            # corners 0 and 2 are outer corners
            corners.pop(1)
        else:
            # corners 1 and 2 are outer corners
            corners.pop(0)
        return min(distance_pacman_to_corner) + short_side_length + long_side_length - 2
    if len(corners) == 2:
        return min(distance_pacman_to_corner) + util.manhattanDistance(*corners)

    # There is only one corner
    return next(distance_pacman_to_corner)


# I had assumed I'd have to use the information about the walls to get a full grade. Turns out, the problem was simpler
# than that. Haha.
#
# class _CornersHeuristicProblemData:
#     """
#     This class uses the Singleton design pattern, and aims to simplify calculations for the cornersHeuristic function
#     by calculating specific values first, and then simply referring to them afterwards.
#
#     THIS CLASS SHOULD NEVER BE INSTANTIATED BY THE USER.
#     """
#
#     _instance = None
#
#     def __init__(self, problem):
#         self.problem = problem
#
#     @classmethod
#     def get_instance(cls, problem):
#         if cls._instance is None or cls._instance.problem is not problem:
#             cls._instance = cls(problem)
#         return cls._instance


class AStarCornersAgent(SearchAgent):
    """A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"""
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem


class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """
    def __init__(self, startingGameState: pacman.GameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0  # DO NOT CHANGE
        self.heuristicInfo = {}  # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        """Returns successor states, the actions they require, and a cost of 1."""
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class AStarFoodSearchAgent(SearchAgent):
    """A SearchAgent for FoodSearchProblem using A* and your foodHeuristic"""
    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem


def foodHeuristic(state: Tuple[Tuple, Grid], problem: FoodSearchProblem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    # Idea: create a minimal spanning tree (MST) between the pellets and pacman. The cost will be the total weight
    # of this tree.
    # I have thought of a marvelous proof for the consistency of this heuristic, but python comments are too narrow to
    # type it out. It is, however, trivial, and as such is left as an exercise for the reader.
    position, food_grid = state

    # Nodes are all pellet locations, plus Pac-man's position
    nodes = food_grid.asList()
    nodes.append(position)

    # Construct graph and apply prim's algorithm
    graph = Graph(nodes)
    graph.apply_prim()

    # return total weight of MST
    return graph.get_total_weight()


def euclidian_distance(xy1, xy2):
    x1, y1 = xy1
    x2, y2 = xy2
    return sqrt((x1-x2)**2 + (y1-y2)**2)


class Graph:
    """
    A Graph which takes in the nodes, and an optional weight function. An adjacency matrix is created, linking every
    node to every other node, and adds a weight to that (undirected) edge equal to the weigh function applied to both
    nodes.

    This Graph class also implements prim's algorithms to convert the graph to a minimal spanning tree of the graph. It
    also implements a method to get the total weight of this (undirected) graph

    This class and relative functions are based on the code at:
    https://stackabuse.com/courses/graphs-in-python-theory-and-implementation/lessons/minimum-spanning-trees-prims-algorithm/
    """
    def __init__(self, nodes, weight_function=euclidian_distance):
        self.node_count = len(nodes)

        # Construct the adjacency matrix
        self.adjacency_matrix = [[0 for _ in range(self.node_count)] for _ in range(self.node_count)]

        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                self.adjacency_matrix[i][j] = weight_function(node1, node2)

    def apply_prim(self):
        """Applies prim's algorithm to this graph, and updates the adjacency matrix accordingly."""
        inf = float("inf")
        selected_nodes = [False for _ in range(self.node_count)]

        result = [[0 for _ in range(self.node_count)] for _ in range(self.node_count)]
        start, end = 0, 0  # start and end indicate the index of the starting and ending node of a new edge

        # oof, look at this nesting... disgusting.
        while False in selected_nodes:
            cheapest = inf

            for i in range(self.node_count):
                if selected_nodes[i]:
                    for j in range(self.node_count):
                        if not selected_nodes[j] and 0 < self.adjacency_matrix[i][j] < cheapest:
                            cheapest = self.adjacency_matrix[i][j]
                            start, end = i, j

            selected_nodes[end] = True
            result[start][end] = cheapest
            result[end][start] = result[start][end]

            # Technically irrelevant, adding for completeness
            # I'd also think this could break the algorithm, since end never updates if this statement is true...?
            if cheapest == inf:
                result[start][end] = 0

        self.adjacency_matrix = result

    def get_total_weight(self):
        return sum(sum(self.adjacency_matrix[i]) for i in range(self.node_count)) / 2


class ClosestDotSearchAgent(SearchAgent):
    """Search for all food using a sequence of searches"""
    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while(currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState) # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception('findPathToClosestDot returned an illegal move: %s!\n%s' % t)
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print('Path found with cost %d.' % len(self.actions))

    def findPathToClosestDot(self, gameState: pacman.GameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        problem = AnyFoodSearchProblem(gameState)
        return search.aStarSearch(problem, minimum_manhattan_heuristic)


class AnyFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to any food.

    This search problem is just like the PositionSearchProblem, but has a
    different goal test, which you need to fill in below.  The state space and
    successor function do not need to be changed.

    The class definition above, AnyFoodSearchProblem(PositionSearchProblem),
    inherits the methods of the PositionSearchProblem.

    You can use this search problem to help you fill in the findPathToClosestDot
    method.
    """

    def __init__(self, gameState):
        """Stores information from the gameState.  You don't need to change this."""
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def isGoalState(self, state: Tuple[int, int]):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x, y = state
        return self.food[x][y]


def minimum_manhattan_heuristic(position, problem: AnyFoodSearchProblem):
    minimum_distance = float("inf")
    x, y = position

    for pellet_location in problem.food.asList():
        x_pellet, y_pellet = pellet_location
        distance_to_pellet = abs(x - x_pellet) + abs(y - y_pellet)
        minimum_distance = min(minimum_distance, distance_to_pellet)

    if minimum_distance == float("inf"):
        return 0  # There are no food pellets left. Assume goal state is different, apply UCS instead (uninformed).
    return minimum_distance


def mazeDistance(point1: Tuple[int, int], point2: Tuple[int, int], gameState: pacman.GameState) -> int:
    """
    Returns the maze distance between any two points, using the search functions
    you have already built. The gameState can be any game state -- Pacman's
    position in that state is ignored.

    Example usage: mazeDistance( (2,4), (5,6), gameState)

    This might be a useful helper function for your ApproximateSearchAgent.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))
