import numpy as np
import time

queue = []


WALL = '#',
BOX  = 'O',
GOAL = 'X',
BOG  = '=', # BOX ON GOAL
PLAYER = 'p',
POG    = 'P' ,# PLAYER ON GOAL
FLOOR  = ' '
direction = [(1,0), (-1,0), (0,1), (0,-1)]
def LoadMap(filename):
    with open(filename) as file:
        map = file.readlines()
        map = [line.rstrip() for line in map]
    return map
MAP = LoadMap('map.txt')
def PosOfPlayer(state):
    for i in range(len(state)):
        if 'p' in state[i]:
            return (i, state[i].index('p'))
        if 'P' in state[i]:
            return (i, state[i].index('P'))
    pass
def PosOfBox(state):
    x = []
    y = []
    list_box = []
    for i in range(len(state)):
        
        if 'O' in state[i]:
            x.append(i)
            y.append(state[i].index('O'))
        if '=' in state[i]:
            x.append(i)
            y.append(state[i].index('='))
    return list(zip(x, y))
def PosOfGoal(state):
    x = []
    y = []
    list_box = []
    for i in range(len(state)):
        
        if 'P' in state[i]:
            x.append(i)
            y.append(state[i].index('P'))
        if 'X' in state[i]:
            x.append(i)
            y.append(state[i].index('X'))
    return list(zip(x, y))
posPlayer = PosOfPlayer(MAP)
posBox = PosOfBox(MAP)
def isSolved(POG, POB):
    return sorted(POG) == sorted(POB)

def availableMove(posPlayer, posBox, direc : direction):
    next = tuple(map(lambda i, j: i + j, posPlayer, direction))
    nextnext = tuple(map(lambda i, j: i + j, next, direction))
    return

print(availableMove(posPlayer, posBox, direc=direction))

def loadmap(filename):
    with open(filename) as file:
        map = file.readlines()
        map = [line.rstrip() for line in map]
    map = [x.replace('\n','') for x in map]
    map = [','.join(map[i]) for i in range(len(map))]
    map = [x.split(',') for x in map]
    return np.array(map)
print(loadmap('map.txt'))

def PosOfPlayer(gameState):
    return tuple(np.argwhere(gameState == 2)[0]) # e.g. (2, 2)

