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

    def __hash__(self):
        return hash((self.x, self.y))

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

    def __repr__(self):
        return '\n'.join([''.join(row) for row in self.map])

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


class stateSet:
    def __init__(self):
        self.walls = set()
        self.goals = set()
        self.boxes = set()
        self.player = Position(0,0)
        self.route = list()
        self.countBOG = 0
    
    def initMap(self, filename):
        map = open(filename).read().split('\n')
        for iRow in range(0, len(map)):
            for iCol in range(0, len(map[iRow])):
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

    def __hash__(self):
        return hash((frozenset(self.boxes), self.player))
        
    def __eq__(self, o: object) -> bool:
        return self.boxes.issubset(o.boxes) and self.player == o.player

    def copy(self):
        other = stateSet()
        other.boxes = deepcopy(self.boxes)
        other.player = deepcopy(self.player)
        other.route = deepcopy(self.route)
        other.walls = self.walls
        other.goals = self.goals
        other.countBOG = self.countBOG
        return other

    def isValidMove(self, direction: Position):
        next = self.player + direction
        if next in self.walls:
            return False
        nextnext = next + direction
        if next in self.boxes and (nextnext in self.boxes or nextnext in self.walls):
            return False
        return True

    def validMoves(self):
        validMoves = []
        for direction in directions:
            if self.isValidMove(direction):
                validMoves.append(direction)
        return validMoves
    
    def push(self, direction: Position):
        posBox = self.player + direction
        posNextOfBox = posBox + direction
        if posBox in self.goals:
            self.countBOG -= 1
        if posNextOfBox in self.goals:
            self.countBOG += 1
        self.boxes.remove(posBox)
        self.boxes.add(posNextOfBox)
        

    def move(self, direction: Position):
        nextState = self.copy()
        nextState.route.append(direction)
        if nextState.player + direction in nextState.boxes:
            nextState.push(direction)
        nextState.player += direction
        return nextState

    def isGoalState(self):
        return self.countBOG == len(self.goals)

def printSolution(initState: State, route, duration):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        time.sleep(0.2)
        nextState = state.move(move)
        if nextState:
            state = nextState
            os.system('cls')
            print(state)
    print("Duration: " + str(duration))    

def BFS(initState): 
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
                    return nextState
                stateQueue.put(nextState)    

filename = 'map2.txt'

initState = State(filename)
print(initState)

initSetState = stateSet()
initSetState.initMap(filename)

start = time.time()
goalState = BFS(initSetState)
end = time.time()

print(len(goalState.route))
printSolution(initState, goalState.route, end - start)

