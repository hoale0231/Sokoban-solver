from queue import Queue
from copy import deepcopy
import time
import os
class Position:
    '''
    Coordinate object
    Support player to move
    '''
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)

# 4 possible moves
DOWN = Position(1,0)
RIGHT = Position(0,1)
UP = Position(-1,0)
LEFT = Position(0,-1)
directions = [DOWN, RIGHT, UP, LEFT]

WALL = '#'
BOX  = 'O'
GOAL = 'X'
BOG  = '='  # BOX ON GOAL
PLAYER = 'p'
POG    = 'P' # PLAYER ON GOAL
FLOOR  = ' '

class MatrixState:
    def __init__(self, filename):
        f = open(filename)
        self.map = f.read().split('\n')
        self.countGoal = 0
        self.countBOG  = 0
        for iRow in range(len(self.map)):
            self.map[iRow] = list(self.map[iRow])
            self.countGoal += self.map[iRow].count(GOAL) + self.map[iRow].count(BOG) + self.map[iRow].count(POG)
            if PLAYER in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(PLAYER))
            if POG in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(POG))

        self.route = []

    def __hash__(self):
        return hash(str(self.map))

    def __eq__(self, o: object):
        return self.map == o.map

    def __repr__(self):
        return '\n'.join([''.join(row) for row in self.map])

    def isValidMove(self, direction):
        next = self.player + direction
        frontNext = self.player + direction + direction
        if self.map[next.x][next.y] == WALL: 
            return False
        if (self.map[next.x][next.y] in {BOX, BOG}) and (self.map[frontNext.x][frontNext.y] in {WALL, BOX, BOG}):
            return False
        return True

    def validMoves(self):
        validMoves = []
        for direction in directions:
            if self.isValidMove(direction):
                validMoves.append(direction)
        return validMoves

    def push(self, direction: Position):
        box = self.player + direction
        nextOfBox = box + direction
        if self.map[nextOfBox.x][nextOfBox.y] == GOAL:
            self.map[nextOfBox.x][nextOfBox.y] = BOG
            self.countBOG += 1
        else:
            self.map[nextOfBox.x][nextOfBox.y] = BOX
        
        if self.map[box.x][box.y] == BOG:
            self.map[box.x][box.y] = GOAL
            self.countBOG -= 1
        else:
            self.map[box.x][box.y] = FLOOR

    def move(self, direction: Position):
        nextState = deepcopy(self)
        nextState.route.append(direction)
        next = nextState.player + direction
        if nextState.map[next.x][next.y] in {BOX, BOG}:
            nextState.push(direction)

        nextState.map[next.x][next.y] = POG if nextState.map[next.x][next.y] == GOAL else PLAYER
        nextState.map[nextState.player.x][nextState.player.y] = GOAL if nextState.map[nextState.player.x][nextState.player.y] == POG else FLOOR
        nextState.player += direction
        return nextState

    def isGoalState(self):
        return self.countBOG == self.countGoal

def printSolution(initState: MatrixState, route, duration):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        time.sleep(0.2)
        state = state.move(move)   
        os.system('cls')
        print(state)
    print("Duration: " + str(duration))

def BFS(initState: MatrixState):
    start = time.time()
    state = deepcopy(initState)
    stateQueue = Queue()
    stateQueue.put(state)
    visited = set()
    visited.add(state)
    while True:
        if stateQueue.empty():
            print("Can't found solution")
            return None
        state = stateQueue.get()
        visited.add(state)
        for move in state.validMoves():
            nextState = state.move(move)
            if nextState not in visited:
                if nextState.isGoalState():
                    end = time.time()
                    printSolution(initState, nextState.route, end - start)
                    return nextState
                stateQueue.put(nextState)

filename = 'map2.txt'

initState = MatrixState(filename)
print(initState)
goalState = BFS(initState)