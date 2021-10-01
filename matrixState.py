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
    '''
    State is a matrix mapped directly from the map
    '''
    def __init__(self):
        self.map = list()           # Matrix 2D
        self.countGoal = 0          # Count number Goal, use to check goal state
        self.countBOG  = 0          # Count number Box on Goal, use to check goal state
        self.player = Position(0,0) # Position of player
        self.route = list()         # Solution route

    # Input map from file
    def initMap(self, filename):
        f = open(filename)
        self.map = f.read().split('\n')
        for iRow in range(len(self.map)):
            self.map[iRow] = list(self.map[iRow])
            self.countGoal += self.map[iRow].count(GOAL) + self.map[iRow].count(BOG) + self.map[iRow].count(POG)
            if PLAYER in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(PLAYER))
            if POG in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(POG))

    # Make the state object hashable, i.e. addable to set()
    def __hash__(self):
        return hash(str(self.map))

    # Make the state object compare, it helps set() to work correctly
    def __eq__(self, o: object):
        return self.map == o.map

    # Help print result
    def __repr__(self):
        return '\n'.join([''.join(row) for row in self.map])

    # Check if the move is valid
    def isLegalMove(self, direction):
        nextPos = self.player + direction
        # Player can't move onto walls
        if self.map[nextPos.x][nextPos.y] == WALL: 
            return False
        # Player can't push the box if there is an obstacle behind
        behindBox = self.player + direction + direction
        if (self.map[nextPos.x][nextPos.y] in {BOX, BOG}) and (self.map[behindBox.x][behindBox.y] in {WALL, BOX, BOG}):
            return False
        # Player can move now!!
        return True

    # Try move each direction in [DOWN, RIGHT, UP, LEFT], return valid next states
    def getValidNextStates(self):
        nextStates = []
        for direction in directions:
            if self.isLegalMove(direction):
                nextState = deepcopy(self)
                nextState.move(direction)
                nextStates.append(nextState)
        return nextStates  

    # Push box, help move function
    def push(self, direction: Position):
        # Update current box position 
        box = self.player + direction
        if self.map[box.x][box.y] == BOG:
            self.map[box.x][box.y] = GOAL
            self.countBOG -= 1
        else:
            self.map[box.x][box.y] = FLOOR
        # Update next box position 
        behindBox = box + direction
        if self.map[behindBox.x][behindBox.y] == GOAL:
            self.map[behindBox.x][behindBox.y] = BOG
            self.countBOG += 1
        else:
            self.map[behindBox.x][behindBox.y] = BOX

    # Move to the next state
    def move(self, direction: Position):
        next = self.player + direction
        # Push box, if its in next position
        if self.map[next.x][next.y] in {BOX, BOG}:
            self.push(direction)
        # Move player
        self.map[next.x][next.y] = POG if self.map[next.x][next.y] == GOAL else PLAYER
        self.map[self.player.x][self.player.y] = GOAL if self.map[self.player.x][self.player.y] == POG else FLOOR
        self.player += direction
        # Update route
        self.route.append(direction)

    def isGoalState(self):
        return self.countBOG == self.countGoal

def printSolution(initState: MatrixState, route, t):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        time.sleep(0.2)
        state.move(move)
        os.system('cls')
        print(state)
    print("Time: " + str(t))
    print("Steps: " + str(len(route)))


# Breadth first search 
def BFS(initState: MatrixState): 
    stateQueue = Queue()
    visited = set()
    stateQueue.put(initState)
    visited.add(initState)
    while not stateQueue.empty():
        state: MatrixState = stateQueue.get() 
        if state.isGoalState():
            return state
        for nextState in state.getValidNextStates():
            if nextState not in visited:
                stateQueue.put(nextState)
                visited.add(nextState) 
    return None

filename = 'map2.txt'

initState = MatrixState()
initState.initMap(filename)
print(initState)

start = time.time()
goalState = BFS(initState)
end = time.time()

if goalState:
    printSolution(initState, goalState.route, end - start)
else:
    print("Can't found solution")