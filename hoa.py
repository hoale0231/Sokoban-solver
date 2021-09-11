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

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)

# 4 possible moves = [DOWN, RIGHT, UP, LEFT]
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

class State:
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

    def __repr__(self):
        return '\n'.join([''.join(row) for row in self.map])

    def getPosition(self):
        return self.player.x, self.player.y

    def isValidMove(self, direction):
        next = self.player + direction
        frontNext = self.player + direction + direction
        if self.map[next.x][next.y] == WALL: 
            return False
        if (self.map[next.x][next.y] in {BOX, BOG}) and (self.map[frontNext.x][frontNext.y] in {WALL, BOX, BOG}):
            return False
        return True

    def push(self, direction: Position):
        box = self.player + direction
        nextOfBox = box + direction
        if self.map[nextOfBox.x][nextOfBox.y] == GOAL:
            self.map[nextOfBox.x][nextOfBox.y] = BOG
            self.countBOG += 1
        else:
            self.map[nextOfBox.x][nextOfBox.y] = BOX
        
        if self.map[box.x][box.y] == BOG:
            self.map[box.x][box.y] == GOAL
            self.countBOG -= 1

    def move(self, direction: Position):
        if not self.isValidMove(direction): return None
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

def printSolution(initState: State, route):
    state = initState
    os.system('cls')
    print(state)
    for move in route:
        time.sleep(0.2)
        os.system('cls')
        nextState = state.move(move)
        if nextState:
            state = nextState
            print(state)

def BFS(initState: State):
    state = deepcopy(initState)
    stateQueue = Queue()
    stateQueue.put(state)
    visited = set()
    while True:
        state = stateQueue.get()
        for direction in directions:
            nextState = state.move(direction)
            if nextState:
                if nextState.isGoalState():
                    return nextState
                stateQueue.put(nextState)



filename = 'map.txt'

initState = State(filename)
print(initState)
goalState = BFS(initState)

input("Press Enter to continue...")
printSolution(initState, goalState.route)