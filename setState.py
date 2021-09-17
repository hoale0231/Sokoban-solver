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

# 4 possible moves = [DOWN, RIGHT, UP, LEFT]
directions = [Position(1,0), Position(0,1), Position(-1,0), Position(0,-1)]

# Convention symbol
WALL = '#'
BOX  = 'O'
GOAL = 'X'
BOG  = '='  # BOX ON GOAL
PLAYER = 'p'
POG    = 'P' # PLAYER ON GOAL
FLOOR  = ' '

class SetState:
    '''
    State are sets of positions of objects on the map
    '''
    def __init__(self):
        self.walls = set()          # Set of walls            
        self.goals = set()          # Set of goals
        self.boxes = set()          # Set of boxes
        self.player = Position(0,0) # Position of player
        self.route = list()         # Solution route
        self.countBOG = 0           # Count number Box on Goal, use to check goal state
    
    # Input map from file
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

    # Make the state object hashable, i.e. addable to set()
    def __hash__(self):
        return hash((frozenset(self.boxes), self.player))
        
    # Make the state object compare, it helps set() to work correctly
    def __eq__(self, o: object) -> bool:
        return self.boxes == o.boxes and self.player == o.player

    # Copy constructor, optimize deepcopy function since set of walls, goals are constant
    def copy(self):
        other = SetState()
        other.boxes = deepcopy(self.boxes)
        other.player = deepcopy(self.player)
        other.route = deepcopy(self.route)
        other.walls = self.walls
        other.goals = self.goals
        other.countBOG = self.countBOG
        return other

    # Check if the move is valid
    def isValidMove(self, direction: Position):
        nextPos = self.player + direction
        # Player can't move onto walls
        if nextPos in self.walls:
            return False
        # Player can't push the box if there is an obstacle behind
        behindBox = nextPos + direction
        if nextPos in self.boxes and (behindBox in self.boxes or behindBox in self.walls):
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

# Breadth first search 
def BFS(initState: SetState): 
    stateQueue = Queue()
    visited = set()
    stateQueue.put(initState)
    visited.add(initState)
    while not stateQueue.empty():
        state: SetState = stateQueue.get() 
        if state.isGoalState():
            return state
        for nextState in state.getValidNextStates():
            if nextState not in visited:
                stateQueue.put(nextState)
                visited.add(nextState) 
    return None

class MatrixState:
    '''
    Use to print solution
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

def printSolution(initState: MatrixState, route, duration):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        time.sleep(0.2)
        state.move(move)
        os.system('cls')
        print(state)
    print("Duration: " + str(duration))    

  

filename = 'map2.txt'

initState = MatrixState(filename)
print(initState)

initSetState = SetState()
initSetState.initMap(filename)

start = time.time()
goalState = BFS(initSetState)
end = time.time()

if goalState:
    print(len(goalState.route))
    printSolution(initState, goalState.route, end - start)
else:
    print("Can't found solution")

