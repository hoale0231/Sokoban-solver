from queue import Queue, PriorityQueue
from copy import deepcopy
import numpy as np
from time import time, sleep
from sys import argv
from os import system

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
    def __init__(self, A_star = False):
        self.walls = set()          # Set of walls            
        self.goals = set()          # Set of goals
        self.boxes = set()          # Set of boxes
        self.player = Position(0,0) # Position of player
        self.route = list()         # Solution route
        self.countBOG = 0           # Count number Box on Goal, use to check goal state
        self.heuristic = 0
        self.deadlock = set()
        self.A_star = A_star
    
    # Input map from file
    def initMap(self, filename, deadlock = True):
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
        if deadlock: self.foundDeadPos(map)

    def foundDeadPos(self, map):
        # Found all position of floor in map include: FLOOR, BOX, PLAYER.
        while True:
            numDeadlock = len(self.deadlock)
            for iRow in range(len(map)):
                for iCol in range(len(map[iRow])):
                    if map[iRow][iCol] in {FLOOR, BOX, PLAYER}:
                        # Check each direction in [DOWN, RIGHT, UP, LEFT]
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
                                self.deadlock.add(pos)
                            # An edge is deadlock if one side is full of walls, and it don't have goal
                            while True: # Do ... While
                                if (pos in self.goals) or (pos + side not in self.walls):
                                    isDeadlock = False
                                pos += front
                                if not isDeadlock or (pos in self.walls) or (pos in self.deadlock):
                                    break
                            # If edge is deadlock, mark it
                            pos = Position(iRow, iCol)
                            while isDeadlock and (pos not in self.walls):
                                self.deadlock.add(pos)
                                pos += front
            self.printmap(deepcopy(map))
            if numDeadlock == len(self.deadlock): break
            else: numDeadlock = len(self.deadlock)

        # Print map after detect deadlock
        
    def printmap(self, map):
        for iRow in range(0, len(map)):
            for iCol in range(len(map[iRow])):
                if Position(iRow, iCol) in self.deadlock:
                    map[iRow][iCol] = '-'
            map[iRow] = ' '.join(map[iRow])
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
    def getMinDist(self, obj, sets):
        return min([ManhattanDistance(obj, element) for element in sets])

    def closestAssignment(self):
        return sum([self.getMinDist(box, self.goals) for box in self.boxes])

    def greedyAssignment(self):
        return 0

    def getHeuristic(self):
        if self.heuristic == 0:
            self.heuristic = len(self.route) * self.A_star+ self.getMinDist(self.player, self.boxes) + self.closestAssignment() 
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
        other.deadlock = self.deadlock
        other.heuristic = 0
        other.A_star = self.A_star
        return other

    # Check if the move is valid
    def isValidMove(self, direction: Position):
        nextPos = self.player + direction
        # Player can't move onto walls
        if nextPos in self.walls:
            return False
        # Player can't push the box if there is an obstacle behind
        behindBox = nextPos + direction
        if nextPos in self.boxes and (behindBox in self.boxes or behindBox in self.walls or behindBox in self.deadlock):
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
        return '\n'.join([' '.join(row) for row in self.map])

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
    filename = 'map' if len(argv) != 2 else argv[1]
    filename = 'map/' + filename + '.txt'
    # Init state
    matrixState = MatrixState(filename) # Use for print solution
    print(matrixState)
    initState = SetState(A_star = False)
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
            choice = int(input("Please choice (1. Blind search 2. Heuristic search 0. Exit): "))
        except ValueError:
            print("Please input 0 or 1 or 2!!!")
            continue

        if choice == 0:
            break
        if choice == 1:
            # printSolution(matrixState, blind.route)
            continue
        if choice == 2:
            printSolution(matrixState, heuristic.route)
            continue
        
        print("Please input 0 or 1 or 2!!!")
