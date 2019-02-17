# myAgents.py
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

from game import Agent
from searchProblems import PositionSearchProblem

import util
import time
import search

"""
IMPORTANT
`agent` defines which agent you will use. By default, it is set to ClosestDotAgent,
but when you're ready to test your own agent, replace it with MyAgent
"""
def createAgents(num_pacmen, agent='MyAgent'):
    return [eval(agent)(index=i) for i in range(num_pacmen)]

food_chasing = util.Queue()
index_chasing = util.Queue()
retarget_lst = []
time = 0
agent_pos = []

class MyAgent(Agent):
    """
    Implementation of your agent.
    """
    def manhattan_dist(self, position1, position2):
        return abs(position1[0]-position2[0])+abs(position1[1]-position2[1])
    def take1(self, elem):
        return elem[1]
    def getAction(self, state):
        # Returns the next action the agent will take
        global agent_pos
        global retarget_lst
        agent_pos[self.index] = state.getPacmanPosition(self.index)

        if retarget_lst[self.index] == 0:
            while self.action_pool.isEmpty() == False:
                trashcan = self.action_pool.pop()
        
        if self.action_pool.isEmpty():
            food = state.getFood().asList()
            self.food = food
            global food_chasing
            global index_chasing
            startPosition = state.getPacmanPosition(self.index)

            for i in range(len(food_chasing.list)):
                j = food_chasing.pop()
                s = index_chasing.pop()
                if j != self.goal:
                    food_chasing.push(j)
                    index_chasing.push(s)

            food_2 = food[:]
            for i in range(len(food)):
                food_2[i] = (i, self.manhattan_dist(food[i], startPosition))
            food_2.sort(key = self.take1)

            #select best goal for this step

            choice_len = len(food)
            if choice_len > 5:
                choice_len = 5

            global time
            time += 1
            flag = True
            for l in range(choice_len):
                i=food_2[l]
                g = food[i[0]]
                dist = i[1]
                t = False
                for j in range(len(food_chasing.list)):
                    if (self.manhattan_dist(food_chasing.list[j], g) <= 3):
                        flag_2 = True
                        if (dist+2>=self.manhattan_dist(g, agent_pos[index_chasing.list[j]])):
                            t = True
                            flag_2 = False
                            break
                        retarget_lst[index_chasing.list[j]] = 0
                if t == True: continue
                flag = False
                break
            if flag == False: self.goal = g
            else:
                self.goal = food[food_2[0][0]]   
                dist = food_2[0][1]

            food_chasing.push(self.goal)
            index_chasing.push(self.index)
            retarget_lst[self.index] = 1

            problem = TheFoodSearchProblem(state, self.index, self.goal)
            action_list = search.aStarSearch(problem, heuristic=self.manhattanHeuristic)
            action_len = len(action_list)
            for j in range(action_len):
                i = action_list[j]
                self.action_pool.push(i)

        action = self.action_pool.pop()
        return action
        #raise NotImplementedError()

    def manhattanHeuristic(self, position, problem, info={}):
        xy1 = position
        xy2 = self.goal
        dist =  abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
        if xy2 in self.food: dist += 10
        return dist
    def euclideanHeuristic(self, position, problem, info={}):
        xy1 = position
        xy2 = self.goal
        dist = ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5 * (-0.4)
        if xy2 in self.food: dist += 10
        return dist
    def initialize(self):
        """
        Intialize anything you want to here. This function is called
        when the agent is first created. If you don't need to use it, then
        leave it blank
        """
        self.action_pool = util.Queue()
        self.goal = (0,0)
        global agent_pos
        agent_pos.append((0,0))
        global retarget_lst
        retarget_lst.append(0)
        "*** YOUR CODE HERE"
        #raise NotImplementedError()

"""
Put any other SearchProblems or search methods below. You may also import classes/methods in
search.py and searchProblems.py. (ClosestDotAgent as an example below)
"""

class ClosestDotAgent(Agent):

    def findPathToClosestDot(self, gameState):
        """
        Returns a path (a list of actions) to the closest dot, starting from
        gameState.
        """
        # Here are some useful elements of the startState
        startPosition = gameState.getPacmanPosition(self.index)
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState, self.index)

        "*** YOUR CODE HERE ***"
        #find the closest dot
        foodLst = food.asList()
        if len(foodLst) == 0: return 0  # Default to trivial solution
        d = 99999
        for food in foodLst:
            tmp_d = mazeDistance(startPosition, food, gameState)
            if (tmp_d < d):
                nextFood = food
                d = tmp_d
        #find path to the closest dot
        prob = PositionSearchProblem(gameState, start=startPosition, goal=nextFood, warn=False, visualize=False)
        return search.bfs(prob)

        # return search.aStarSearch(problem)
        # return search.uniformCostSearch(problem)

        util.raiseNotDefined()

    def getAction(self, state):
        return self.findPathToClosestDot(state)[0]

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

    def __init__(self, gameState, agentIndex):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()

        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state

        "*** YOUR CODE HERE ***"
        isDest = state == self.goal
        noFood = not self.food.count()
        return isDest and noFood

        util.raiseNotDefined()

class TheFoodSearchProblem(PositionSearchProblem):
    """
    A search problem for finding a path to a specific food.
    """
    def __init__(self, gameState, agentIndex, goal):
        "Stores information from the gameState.  You don't need to change this."
        # Store the food for later reference
        self.food = gameState.getFood()
        self.goal = goal
        # Store info for the PositionSearchProblem (no need to change this)
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition(agentIndex)
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0 # DO NOT CHANGE

    def isGoalState(self, state):
        """
        The state is Pacman's position. Fill this in with a goal test that will
        complete the problem definition.
        """
        x,y = state
        "*** YOUR CODE HERE ***"
        if x == self.goal[0] and y == self.goal[1]:
            return True
        else:
            return False
        util.raiseNotDefined()