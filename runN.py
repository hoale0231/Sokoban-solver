from queue import Queue, PriorityQueue
from copy import deepcopy
import numpy as np
from time import time, sleep
from sys import argv
from os import system, listdir
from multiprocessing import Process, Manager
from pandas import DataFrame

class Position:
    '''
    Coordinate object
    Support player to move
    '''
    def __init__(self, row, col):
        self.row = row
        self.col = col

    # Make the position object hashable, i.e. addable to set()
    def __hash__(self):
        return hash((self.row, self.col))
    
    # Make the state object comparable, it helps set() to work correctly
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    # Help player move
    def __add__(self, other):
        return Position(self.row + other.row, self.col + other.col)

    # Greedy algorithm uses PriorityQueue <(Int, Position, Position)>, this function helps PriorityQueue skip comparing 2 object positions
    def __lt__(self, other):
        return False

def ManhattanDistance(P1: Position, P2: Position):
    return abs(P1.row - P2.row) + abs(P1.col - P2.col)

# 4 possible moves = [DOWN, RIGHT, UP, LEFT]
DIRECTIONS = [Position(1,0), Position(0,1), Position(-1,0), Position(0,-1)]

# Convention symbol
WALL = '#'
BOX  = 'O'
GOAL = 'X'
BOG  = '='  # BOX ON GOAL
PLAYER = 'p'
POG    = 'P' # PLAYER ON GOAL
FLOOR  = ' '
DEADLOCK = '-'
POD   = '+' # PLAYER ON DEADLOCK

class SetState:
    '''
    State are sets of positions of objects on the map
    '''
    def __init__(self):
        self.walls = set()              # Set of walls            
        self.goals = set()              # Set of goals
        self.boxes = set()              # Set of boxes
        self.player = Position(0,0)     # Position of player
        self.countBOG = 0               # Count number Box on Goal, use to check goal state
        self.deadlocks = set()          # Set of deadlocks position
        self.route = list()             # Solution route
        self.heuristic = 0              # Heuristic value    
    
    # Static member, use to set mode heuristic, deadlock detection
    greedy = True   # True: greedy assigment | False: Closest assignment 
    optimal = True  # True: get optimal heuristic solution (i.e. A* Search) | False: Best-First Search
    detectDeadlock = True # Detect deadlock?
    
    @staticmethod
    def setMode(greedy, optimal, detectDeadlock):
        SetState.greedy = greedy
        SetState.optimal = optimal
        SetState.detectDeadlock = detectDeadlock

    # Input map from file
    def initMap(self, filename):
        map = open(filename).read().split('\n')
        for iRow in range(len(map)):
            map[iRow] = list(map[iRow])
            for iCol in range(len(map[iRow])):
                pos = Position(iRow, iCol)
                char = map[iRow][iCol]
                if char == FLOOR:
                    continue
                elif char == WALL:
                    self.walls.add(pos)
                elif char == BOX:
                    self.boxes.add(pos)
                elif char == GOAL:
                    self.goals.add(pos)
                elif char == PLAYER:
                    self.player = pos
                elif char == BOG:
                    self.boxes.add(pos)
                    self.goals.add(pos)
                    self.countBOG += 1
                elif char == POG:
                    self.goals.add(pos)
                    self.player = pos
        self.foundDeadPos(map)
        return map

    def foundDeadPos(self, map):
        # Repeat until no more deadlocks can be found, Do ... While ...
        while True:
            nDeadlock = len(self.deadlocks)
            # Found all position of floor in map include: FLOOR, BOX, PLAYER.
            for iRow in range(len(map)):
                firstCol = 0
                while map[iRow][firstCol] != WALL: firstCol += 1
                for iCol in range(firstCol, len(map[iRow])):
                    if map[iRow][iCol] in {FLOOR, BOX, PLAYER}:
                        # Check each direction in [DOWN, RIGHT, UP, LEFT] & [LEFT, UP, RIGHT, DOWN]
                        for directions in [DIRECTIONS, DIRECTIONS[::-1]]:
                            for direction in range(len(directions)):
                                '''
                                A position is deadlocked if it has both 2 adjacent sides are walls
                                Choose one side is 'back' and the other is 'side' 
                                Assign:
                                    front = directions[direction]
                                    side = directions[direction - 1]
                                    back = directions[direction - 2]
                                Because Directions = [DOWN, RIGHT, UP, LEFT], so:
                                ---------------------
                                Front | Side  | Back
                                ---------------------
                                DOWN  | LEFT  | UP
                                RIGHT | DOWN  | LEFT
                                UP    | RIGHT | DOWN
                                LEFT  | UP    | RIGHT
                                ---------------------
                                '''
                                pos = Position(iRow, iCol)
                                front = directions[direction]
                                side = directions[direction - 1]
                                back = directions[direction - 2]
                                # A position is deadlock if it has 2 adjacent sides of walls
                                isDeadlock = (pos + back in self.walls) and (pos + side in self.walls)
                                if isDeadlock: 
                                    self.deadlocks.add(pos)
                                # A line is deadlock if one side is full of walls, and it don't have goal
                                while True: # Do ... While ...
                                    if (pos in self.goals) or (pos + side not in self.walls):
                                        isDeadlock = False
                                    pos += front
                                    if not isDeadlock or (pos in self.walls) or (pos in self.deadlocks):
                                        break
                                # If edge is deadlock, mark it
                                pos = Position(iRow, iCol)
                                pos += front
                                while isDeadlock and (pos not in self.walls) and (pos not in self.deadlocks):
                                    self.deadlocks.add(pos)
                                    pos += front
            if nDeadlock == len(self.deadlocks): break
            else: nDeadlock = len(self.deadlocks)
        for iRow in range(0, len(map)):
            for iCol in range(len(map[iRow])):
                if Position(iRow, iCol) in self.deadlocks:
                    map[iRow][iCol] = POD if Position(iRow, iCol) == self.player else DEADLOCK
        


    # Make the state object hashable, i.e. addable to set()
    def __hash__(self):
        return hash((frozenset(self.boxes), self.player))
        
    # Make the state object comparable, it helps set(), PriorityQueue() to work correctly
    def __eq__(self, o):
        return self.boxes == o.boxes and self.player == o.player
    def __gt__(self, o):
        return self.getHeuristic() > o.getHeuristic()
    def __lt__(self, o):
        return self.getHeuristic() < o.getHeuristic()

    def getMinDist(self, obj, sets):
        return min([ManhattanDistance(obj, element) for element in sets]) 

    def closestAssignment(self):
        return sum([self.getMinDist(box, self.goals) for box in self.boxes])

    def goalPullMetric(self):
        # This function help find all the cost from all the boxes when move to all the goals
        result = PriorityQueue()
        for goal in self.goals:
            for box in self.boxes:
                result.put((ManhattanDistance(goal, box), goal, box))
        return result

    def closestGoal(self, position: Position, boxSet):
        # find closest box for a goal base on pythagorean distance
        distanceVal = set()
        for box in boxSet:
            minBox: Position(0, 0)
            distance = ManhattanDistance(position, box)
            distanceVal.add(distance)
            if distance == min(distanceVal):
                minBox = box
        return (minBox, distance)

    def greedyAssignment(self):
        # This function assign each box to each goal
        goalboxqueue = self.goalPullMetric()
        matchedBoxes = set()
        matchedGoals = set()
        #match = set()
        totalPath = 0
        while not goalboxqueue.empty():
            (p, g, b) = goalboxqueue.get()
            if g not in matchedGoals and b not in matchedBoxes:
                matchedGoals.add(g)
                matchedBoxes.add(b)
                totalPath += p

        notAssignedBox = set()
        for b in self.boxes:
            if b not in matchedBoxes:
                notAssignedBox.add(b)

        for g in self.goals:
            if g not in matchedGoals:
                b = self.closestGoal(g, notAssignedBox)
                notAssignedBox.remove(b[0])
                totalPath += b[1]
        return totalPath

    def getHeuristic(self):
        if self.heuristic == 0:
            self.heuristic = len(self.route) * SetState.optimal \
                            + (self.greedyAssignment() if SetState.greedy else self.closestAssignment()) \
                            + self.getMinDist(self.player, self.boxes) * (1 - SetState.optimal) # Applies only to Best-First Search, because it's not an admissible heuristic
        return self.heuristic

    # Copy constructor, optimize deepcopy function since set of walls, goals are constant
    def copy(self):
        other = SetState()
        other.boxes = deepcopy(self.boxes)
        other.player = deepcopy(self.player)
        other.route = deepcopy(self.route)
        other.walls = self.walls
        other.goals = self.goals
        other.countBOG = self.countBOG
        other.deadlocks = self.deadlocks
        other.heuristic = 0
        return other

    # Check if the move is valid
    def isValidMove(self, direction: Position):
        nextPos = self.player + direction
        # Player can't move onto walls
        if nextPos in self.walls:
            return False
        # Check behind box
        behindBox = nextPos + direction
        if nextPos in self.boxes:
             # Player can't push the box if there is an obstacle behind
            if behindBox in self.boxes or behindBox in self.walls:
                return False
             # Player shouldn't push the box if behind is the deadlock position
            if SetState.detectDeadlock and behindBox in self.deadlocks:
                return False
        # Player can move now!!
        return True

    # Try move each direction in [DOWN, RIGHT, UP, LEFT], return valid next states
    def getValidNextStates(self):
        nextStates = []
        for direction in DIRECTIONS:
            if self.isValidMove(direction):
                nextState = self.copy()
                nextState.move(direction)
                nextStates.append(nextState)
        return nextStates  

    # Move to the next state
    def move(self, direction: Position):
        nextPos = self.player + direction
        # Push box, if its in next position
        if nextPos in self.boxes:
            behindBox = nextPos + direction
            # Push box
            self.boxes.remove(nextPos)
            self.boxes.add(behindBox)
            # Update countPOG
            if nextPos in self.goals:
                self.countBOG -= 1
            if behindBox in self.goals:
                self.countBOG += 1
        # Move player
        self.player = nextPos
        # Update route
        self.route.append(direction)

    def isGoalState(self):
        return self.countBOG == len(self.goals)

'''
Search algorithm
Blind       = Search(initState, Queue())
Heuristic   = Search(initState, PriorityQueue())
'''
def Search(initState: SetState, queue: Queue): 
    stateQueue = queue
    visited = set()
    stateQueue.put(initState)
    visited.add(initState)
    cnt = 0
    while not stateQueue.empty():
        state: SetState = stateQueue.get() 
        if state.isGoalState():
            return state, len(visited), cnt
        for nextState in state.getValidNextStates():
            cnt += 1
            if nextState not in visited:
                stateQueue.put(nextState)
                visited.add(nextState) 
    return None



########################################################################################################
########################################################################################################
###################################### STOP HERE !!! ###################################################
########################################################################################################
########################################################################################################

