import numpy as np
from queue import Queue
import time

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
    map = [','.join(map[i]) for i in range(len(map))]
    map = [x.split(',') for x in map]
    return np.array(map)
MAP = LoadMap('map.txt')
#get position
def PosOfPlayer(state):
    for x in np.argwhere((state == PLAYER)):
        return tuple(x)

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

def isValidMove(pop, pob, pow, direc):
    nextx, nexty = pop[0] + direc[0], pop[1] + direc[1]
    nextnextx, nextnexty = nextx + direc[0], nexty + direc[1]
    if (nextx, nexty) in pow:
        return False
    elif ((nextx, nexty) in pob) and ((nextnextx, nextnexty) in pow + pob):
        return False
    return (nextx, nexty)

def setMove(pop, pob, pow):
    setM = []
    for direc in directions:
        if (isValidMove(pop, pob ,pow, direc) != False):
            setM.append(direc)
    return setM

def update(pop, pob, direc):
    x, y = pop
    temp_pob = list(tuple(pob))
    nextpop = (x+direc[0], y + direc[1])
    if nextpop in temp_pob:
        nextnextpop = (x+direc[0]*2, y+direc[1]*2)
        temp_pob.remove(nextpop)
        temp_pob.append(nextnextpop)
    newpop = tuple(nextpop)
    newpob = temp_pob
    return newpop, newpob 

def isGoalState(pob, pog):
    return sorted(pob) == sorted(pog)

def BFS(pop, pob):
    start = time.time()
    route = []
    initial_state = (pop, pob, route)
    stateQueue = Queue()
    stateQueue.put(initial_state)
    visited = []
    visited.append(initial_state)
    while True:
        if stateQueue.empty():
            print("Can't found solution")
            return None
        state = stateQueue.get()
        temp_route = list(tuple(state[2]))
        visited.append(state)
        for direc in setMove(state[0], state[1], pow):
            temp_route.append(direc)
            temp_pop, temp_pob = update(state[0], state[1], direc)
            nextState = (temp_pop, temp_pob, temp_route)
            if nextState not in visited:
                if isGoalState(nextState[1], pog):
                    end = time.time()
                    print("Runtime of BFS: ", end - start)
                    return nextState[2]
                stateQueue.put(nextState)
            temp_route = list(tuple(state[2]))
        
if __name__ == '__main__':
    MAP = LoadMap('map.txt')
    #print(MAP)
    pop = PosOfPlayer(MAP)
    pob = PosOfBoxes(MAP)
    pog = PosOfGoals(MAP)
    pow = PosOfWalls(MAP)

    result = (BFS(pop, pob))
    str = ""
    for sol in result:
        if (sol == UP):
            str += 'U'
        if (sol == DOWN):
            str += 'D'
        if (sol == RIGHT):
            str += 'R'
        if (sol == LEFT):
            str += 'L'
        str += '|'
    print("Solution: ",str)
