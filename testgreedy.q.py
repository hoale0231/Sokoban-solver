from queue import Queue, PriorityQueue
from copy import deepcopy
import numpy as np
from time import time, sleep
from sys import argv
from os import system
from hug import hungarian_algorithm


class Position:
    '''
    Coordinate object
    Support player to move
    '''

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)


def ManhattanDistance(P1: Position, P2: Position):
    return abs(P1.x - P2.x) + abs(P1.y - P2.y)


def PythagoreanDistance(P1: Position, P2: Position):
    return np.sqrt((P1.x - P2.x)**2 + (P1.y - P2.y)**2)


# 4 possible moves = [DOWN, RIGHT, UP, LEFT]
directions = [Position(1, 0), Position(0, 1), Position(-1, 0), Position(0, -1)]

# Convention symbol
WALL = '#'
BOX = 'O'
GOAL = 'X'
BOG = '='  # BOX ON GOAL
PLAYER = 'p'
POG = 'P'  # PLAYER ON GOAL
FLOOR = ' '


class SetState:
    '''
    State are sets of positions of objects on the map
    '''

    def __init__(self):
        self.walls = set()          # Set of walls
        self.goals = set()          # Set of goals
        self.boxes = set()          # Set of boxes
        self.player = Position(0, 0)  # Position of player
        self.route = list()         # Solution route
        self.countBOG = 0           # Count number Box on Goal, use to check goal state
        self.heuristic = 0
        self.deadPos = set()

    # Input map from file
    def initMap(self, filename, deadlock=True):
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
        if deadlock:
            self.foundDeadPos(map)

    def foundDeadPos(self, map):
        for iRow in range(len(map)):
            for iCol in range(len(map[iRow])):
                if map[iRow][iCol] in {FLOOR, BOX, PLAYER} and Position(iRow, iCol) not in self.deadPos:
                    for ifront in range(len(directions)):
                        pos = Position(iRow, iCol)
                        front = directions[ifront]
                        side = directions[ifront - 1]
                        back = directions[ifront - 2]
                        flag = pos + back in self.walls and pos + side in self.walls
                        if flag:
                            self.deadPos.add(pos)
                        while flag and pos not in self.walls:
                            if pos in self.goals or pos + side not in self.walls:
                                flag = False
                            pos += front
                        pos = Position(iRow, iCol)
                        while flag and pos not in self.walls:
                            self.deadPos.add(pos)
                            pos += front
        for iRow in range(0, len(map)):
            for iCol in range(len(map[iRow])):
                if Position(iRow, iCol) in self.deadPos:
                    map[iRow][iCol] = '.'
            map[iRow] = ''.join(map[iRow])
        print("Map after detect deadlock")
        print('\n'.join(map))

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

    ####################################################################################
    ####################################################################################
    ####################################################################################
    '''
    Define heuristic function here!!!
    U can use ManhattanDistance(P1: Position, P2: Position) 
           or PythagoreanDistance(P1: Position, P2: Position) to get distance of 2 object
    '''

    def getMinDistPlayerToBox(self):
        m = 100000000
        for box in self.boxes:
            m = min(PythagoreanDistance(self.player, box), m)
        return m - 1

    # help Functions
    def goalPullMetric(self):
        # This function help find all the cost from all the boxes when move to all the goals
        for goal in self.goals:
            result = set()
            for box in self.boxes:
                # check if goal and box is possible
                direction = Position((box.x - goal.x), (box.y-goal.y))
                if direction.x > 0 and (box.x+1) in self.wall:
                    continue
                if direction.x < 0 and (box.x-1) in self.wall:
                    continue
                if direction.y > 0 and (box.y+1) in self.wall:
                    continue
                if direction.y < 0 and (box.y-1) in self.wall:
                    continue
                elif direction.x == 0:
                    result.add((abs(direction.y), goal, box))
                elif direction.y == 0:
                    result.add((abs(direction.x), goal, box))
                else:
                    dis = abs(direction.x) + abs(direction.y)
                    result.add(dis, goal, box)
                # cach khac?
        # return priority-(u,v)

    def closestGoal(position: Position, boxSet):
        # find closest box for a goal base on pythagorean distance
        distanceVal = set()
        for box in boxSet:
            minBox: Position(0, 0)
            distance = PythagoreanDistance(position, box)
            distanceVal.add(distance)
            if distance == min(distanceVal):
                minBox = box
        return (minBox, distance)

    def greedyAssignment(self):
        # This function assign each box to each goal
        goalboxqueue = PriorityQueue
        goalboxqueue = goalPullMetric(self)
        matchedBoxes = set()
        matchedGoals = set()
        #match = set()
        totalPath = 0
        while len(goalboxqueue) != 0:
            (p, g, b) = goalboxqueue.pop()
            if not matchedGoals.contains(goalboxqueue) and not matchBoxes.contains(b):
                matchedGoals.add(g)
                matchedBoxes.add(b)
                #match.add((g, b))
                totalPath += p
        notAssignedBox = set()
        for b in self.boxes:
            if not matchedBoxes.contains(b):
                notAssignedBox.add(b)

        for g in self.goals:
            if not matchedGoals.contains(g):
                b = closestGoal(g, notAssignedBox)
                notAssignedBox.remove(b[0])
                totalPath += b[1]
                #match.add((g, b))
        return totalPath

    def closestAssignment(self):
        return 0

    def HungarianAssignment(self):
        mat = np.array([[PythagoreanDistance(x, y)
                       for x in self.boxes] for y in self.goals])
        return hungarian_algorithm(mat)

    def getHeuristic(self):
        if self.heuristic == 0:
            self.heuristic = len(self.route) + self.greedyAssignment()
        return self.heuristic
    ####################################################################################
    ####################################################################################
    ####################################################################################

    # Copy constructor, optimize deepcopy function since set of walls, goals are constant
    def copy(self):
        other = SetState()
        other.boxes = deepcopy(self.boxes)
        other.player = deepcopy(self.player)
        other.route = deepcopy(self.route)
        other.walls = self.walls
        other.goals = self.goals
        other.countBOG = self.countBOG
        other.deadPos = self.deadPos
        other.heuristic = 0
        return other

    # Check if the move is valid
    def isValidMove(self, direction: Position):
        nextPos = self.player + direction
        # Player can't move onto walls
        if nextPos in self.walls:
            return False
        # Player can't push the box if there is an obstacle behind
        behindBox = nextPos + direction
        if nextPos in self.boxes and (behindBox in self.boxes or behindBox in self.walls or behindBox in self.deadPos):
            return False
        # Player can move now!!
        return True

    # Try move each direction in [DOWN, RIGHT, UP, LEFT], return valid next states
    def getValidNextStates(self):
        nextStates = []
        for direction in directions:
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
BFS = Search(initState, Queue())
A*  = Search(initState, PriorityQueue())
'''


def Search(initState: SetState, queue: PriorityQueue):
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


class MatrixState:
    '''
    Use to print solution
    Don't care me!!
    '''

    def __init__(self, filename):
        f = open(filename)
        self.map = f.read().split('\n')
        for iRow in range(len(self.map)):
            self.map[iRow] = list(self.map[iRow])
            if PLAYER in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(PLAYER))
            if POG in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(POG))

    def __repr__(self):
        return '\n'.join([''.join(row) for row in self.map])

    def push(self, direction: Position):
        box = self.player + direction
        nextOfBox = box + direction
        if self.map[nextOfBox.x][nextOfBox.y] == GOAL:
            self.map[nextOfBox.x][nextOfBox.y] = BOG
        else:
            self.map[nextOfBox.x][nextOfBox.y] = BOX

        if self.map[box.x][box.y] == BOG:
            self.map[box.x][box.y] = GOAL
        else:
            self.map[box.x][box.y] = FLOOR

    def move(self, direction: Position):
        next = self.player + direction
        if self.map[next.x][next.y] in {BOX, BOG}:
            self.push(direction)

        self.map[next.x][next.y] = POG if self.map[next.x][next.y] == GOAL else PLAYER
        self.map[self.player.x][self.player.y] = GOAL if self.map[self.player.x][self.player.y] == POG else FLOOR
        self.player += direction


def printSolution(initState: MatrixState, route):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        sleep(0.2)
        state.move(move)
        system('cls')
        print(state)


if __name__ == '__main__':
    # Input map
    filename = 'map.txt' if len(argv) != 2 else argv[1]

    # Init state
    matrixState = MatrixState(filename)  # Use for print solution
    print(matrixState)
    initState = SetState()
    initState.initMap(filename)

    # Blind search
    start = time()
    blind, nblind, cntblind = Search(initState, Queue())
    end = time()
    blindTime = end - start

    # Heuristic search
    start = time()
    heuristic, nheur, cntheu = Search(initState, PriorityQueue())
    end = time()
    heuristicTime = end - start

    # Print solution
    print("Solution is ready!!")
    while True:
        print("Blind search")
        print("Duration:", blindTime)
        print("Step:", len(blind.route))
        print("Node visited:", nblind)
        print("Nodes generated:", cntblind)
        print("\nHeuristic search")
        print("Duration:", heuristicTime)
        print("Step:", len(heuristic.route))
        print("Node visited:", nheur)
        print("Nodes generated:", cntheu)

        try:
            choice = int(
                input("Please choice (1. Blind search 2. Heuristic search 0. Exit): "))
        except ValueError:
            print("Please input 0 or 1 or 2!!!")
            continue

        if choice == 0:
            break
        if choice == 1:
            printSolution(matrixState, blind.route)
            continue
        if choice == 2:
            printSolution(matrixState, heuristic.route)
            continue

        print("Please input 0 or 1 or 2!!!")
