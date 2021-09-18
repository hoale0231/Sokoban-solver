import numpy as np
import queue
from copy import deepcopy

WALL = '#',
BOX  = 'O',
GOAL = 'X',
BOG  = '=', # BOX ON GOAL
PLAYER = 'p',
POG    = 'P' ,# PLAYER ON GOAL
FLOOR  = ' '

UP = (-1,0)
DOWN = (1,0)
RIGHT =  (0,1)
LEFT =  (0,-1)
directions = [(-1,0), (1,0), (0,1), (0,-1)]



def LoadMap(filename):
    with open(filename) as file:
        map = file.readlines()
        map = [line.rstrip() for line in map]
    #map = [x.replace('\n','') for x in map]
    map = [','.join(map[i]) for i in range(len(map))]
    map = [x.split(',') for x in map]
    return np.array(map)

MAP = LoadMap('map.txt')

#get position
def PosOfPlayer(state):
    return tuple(np.argwhere(state == PLAYER)[0])
def PosOfBoxes(state):
    temp = []
    for x in np.argwhere((state == BOX) | (state == BOG)):
        temp.append(tuple(x))
    return temp
def PosOfGoals(state):
    temp = []
    for x in np.argwhere((state == GOAL) | (state == BOG)):
        temp.append(tuple(x))
    return temp
def PosOfWalls(state):
    temp = []
    for x in np.argwhere(state == WALL):
        temp.append(tuple(x))
    return temp

def isValidMove(pop, pow, pob, direction):
    next = tuple(map(lambda i, j: i + j, pop, direction))
    nextnext = tuple(map(lambda i, j: i + j, next, direction))
    if next in pow:
        return False
    elif (next in pob) and (nextnext in pow + pob):
        return False
    return next, nextnext

def update(pop, pow, pob, direction):
    if (isValidMove(pop, pow, pob, direction) == False):
        return False
    nextpop, nextnextpop = isValidMove(pop, pow, pob, direction)
    if nextpop in pob:
        pob.remove(nextpop)
        pob.append(nextnextpop)
    pop = tuple(nextpop)
    return pop, pob  
def isGoalState(pob, pog):
    return sorted(pog) == sorted(pob)

def printSol(initial_state):

    return

def BFS(pop, pob):
    route = []
    initial_state = (pop, pob)
    state = deepcopy(initial_state)
    stateQueue = queue.Queue()
    stateQueue.put(state)
    visited = []
    visited.append(state)
    found = False
    while not found:
        if stateQueue.empty():
            print("Not found!")
            return None
        state = stateQueue.get()
        visited.append(state)
        for direc in directions:
            print(direc)
            pop, pob = update(pop, pow, pob, direc)
            if (pop, pob) not in visited:
                if (isGoalState(pop, pob)):
                    print("Found!")
                    return route
                stateQueue.put(pop, pob)

if __name__ == '__main__':
    MAP = LoadMap('map.txt')
    #print(MAP)
    pop = PosOfPlayer(MAP)
    pob = PosOfBoxes(MAP)
    pog = PosOfGoals(MAP)
    pow = PosOfWalls(MAP)
    #print("list of pos of player", pop)
    #print("list of pos of boxes", pob)
    #print("list of pos of goals", pog)
    #print("list of pos of walls", pow)

    #move = [DOWN, LEFT, DOWN, RIGHT]
    #for m in move:
    #    pop, pob = update(pop, pow, pob, m)
    #print(isGoalState(pob, pog))

    #print((isValidMove(pop, pow, pob, direction[1])))
    #print((isValidMove(pop, pow, pob, direction[1])))
    print(BFS(pop, pob))
