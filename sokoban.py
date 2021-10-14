from queue import Queue, PriorityQueue
from copy import deepcopy
from time import time, sleep
from sys import argv
import pygame

class Position:
    def __init__(self, row, col):
        self.row = row
        self.col = col

    # Make the position object hashable, i.e. addable to set()
    def __hash__(self):
        return hash((self.row, self.col))
    
    # Make the state object comparable, it helps set() to work correctly
    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    # Help player move
    def __add__(self, other):
        return Position(self.row + other.row, self.col + other.col)

    # Greedy algorithm uses PriorityQueue <(Int, Position, Position)>, this function helps PriorityQueue skip comparing 2 object positions
    def __lt__(self, other):
        return False

def ManhattanDistance(P1: Position, P2: Position):
    return abs(P1.row - P2.row) + abs(P1.col - P2.col)

# 4 possible moves = [DOWN, RIGHT, UP, LEFT]
DIRECTIONS = [Position(1,0), Position(0,1), Position(-1,0), Position(0,-1)]

# Convention symbol
WALL = '#'
BOX  = 'O'
GOAL = 'X'
BOG  = '='  # BOX ON GOAL
PLAYER = 'p'
POG    = 'P' # PLAYER ON GOAL
FLOOR  = ' '
DEADLOCK = '-'
POD   = '+' # PLAYER ON DEADLOCK

class SetState:
    '''
    State are sets of positions of objects on the map
    '''
    def __init__(self):
        self.walls = set()              # Set of walls            
        self.goals = set()              # Set of goals
        self.boxes = set()              # Set of boxes
        self.player = Position(0,0)     # Position of player
        self.countBOG = 0               # Count number Box on Goal, use to check goal state
        self.deadlocks = set()          # Set of deadlocks position
        self.route = list()             # Solution route
        self.heuristic = 0              # Heuristic value    
    
    # Input map from file
    def initMap(self, filename):
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
        self.foundDeadPos(map)
        return map

    # Deadlock detection
    def foundDeadPos(self, map):
        # Found all position of floor in map include: FLOOR, BOX, PLAYER.
        for iRow in range(len(map)):
            firstCol = 0
            while map[iRow][firstCol] != WALL: firstCol += 1
            for iCol in range(firstCol, len(map[iRow])):
                if map[iRow][iCol] in {FLOOR, BOX, PLAYER}:
                    # Check each direction in [DOWN, RIGHT, UP, LEFT] & [LEFT, UP, RIGHT, DOWN]
                    for directions in [DIRECTIONS, DIRECTIONS[::-1]]:
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
                                self.deadlocks.add(pos)
                            # A line is deadlock if one side is full of walls, and it don't have goal
                            while True: # Do ... While ...
                                if (pos in self.goals) or (pos + side not in self.walls):
                                    isDeadlock = False
                                pos += front
                                if not isDeadlock or (pos in self.walls):
                                    break
                            # If edge is deadlock, mark it
                            pos = Position(iRow, iCol)
                            pos += front
                            while isDeadlock and (pos not in self.walls):
                                self.deadlocks.add(pos)
                                pos += front
        # Change the map, help for print the solution   
        for iRow in range(0, len(map)):
            for iCol in range(len(map[iRow])):
                if Position(iRow, iCol) in self.deadlocks:
                    map[iRow][iCol] = POD if Position(iRow, iCol) == self.player else DEADLOCK
    
    # Make the state object comparable, it helps set(), PriorityQueue() to work correctly
    def __eq__(self, o):
        return self.boxes == o.boxes and self.player == o.player
    def __gt__(self, o):
        return self.getHeuristic() > o.getHeuristic()
    def __lt__(self, o):
        return self.getHeuristic() < o.getHeuristic()

    # Make the state object hashable, i.e. addable to set()
    def __hash__(self):
        return hash((frozenset(self.boxes), self.player))

    # Copy constructor, optimize deepcopy function since set of walls, goals are fixed postion
    def copy(self):
        other = SetState()
        other.boxes = deepcopy(self.boxes)
        other.player = deepcopy(self.player)
        other.route = deepcopy(self.route)
        other.walls = self.walls
        other.goals = self.goals
        other.countBOG = self.countBOG
        other.deadlocks = self.deadlocks
        other.heuristic = 0
        return other

    # Check if the move is valid
    def isValidMove(self, direction: Position):
        nextPos = self.player + direction
        # Player can't move onto walls
        if nextPos in self.walls:
            return False
        # Check behind box
        behindBox = nextPos + direction
        if nextPos in self.boxes:
             # Player can't push the box if there is an obstacle behind
            if behindBox in self.boxes or behindBox in self.walls:
                return False
             # Player shouldn't push the box if behind is the deadlock position
            if SetState.detectDeadlock and behindBox in self.deadlocks:
                return False
        # Player can move now!!
        return True

    # Try move each direction in [DOWN, RIGHT, UP, LEFT], return valid next states
    def getValidNextStates(self):
        nextStates = []
        for direction in DIRECTIONS:
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

    # Check if a state is goal state 
    def isGoalState(self):
        return self.countBOG == len(self.goals)
    
    '''
    Behind methods help to calculate the heuristic value
    '''
    # Static member, use to set mode heuristic, deadlock detection
    greedy = True   # True: greedy assigment | False: Closest assignment 
    optimal = True  # True: get optimal heuristic solution (i.e. A* Search) | False: Best-First Search
    detectDeadlock = True # Detect deadlock?

    @staticmethod
    def setMode(greedy, optimal, detectDeadlock):
        SetState.greedy = greedy
        SetState.optimal = optimal
        SetState.detectDeadlock = detectDeadlock

    def getHeuristic(self):
        if self.heuristic == 0:
            self.heuristic = len(self.route) * SetState.optimal \
                            + (self.greedyAssignment() if SetState.greedy else self.closestAssignment()) \
                            + self.getMinDist(self.player, self.boxes) * (1 - SetState.optimal) # Applies only to Best-First Search, because it's not an admissible heuristic
        return self.heuristic

    '''
    Closest Assignment
    '''
    def getMinDist(self, obj, sets):
        return min([ManhattanDistance(obj, element) for element in sets]) 

    def closestAssignment(self):
        return sum([self.getMinDist(box, self.goals) for box in self.boxes])

    '''
    Greedy Assignment
    '''
    # This function help find all the cost from all the boxes when move to all the goals
    def goalPullMetric(self):
        result = PriorityQueue()
        for goal in self.goals:
            for box in self.boxes:
                result.put((ManhattanDistance(goal, box), goal, box))
        return result

    # Find closest box for a goal base on pythagorean distance
    def closestGoal(self, position: Position, boxSet): 
        distanceVal = set()
        for box in boxSet:
            minBox: Position(0, 0)
            distance = ManhattanDistance(position, box)
            distanceVal.add(distance)
            if distance == min(distanceVal):
                minBox = box
        return (minBox, distance)

    # This function assign each box to each goal
    def greedyAssignment(self):    
        goalboxqueue = self.goalPullMetric()
        matchedBoxes = set()
        matchedGoals = set()
        totalPath = 0
        while not goalboxqueue.empty():
            (p, g, b) = goalboxqueue.get()
            if g not in matchedGoals and b not in matchedBoxes:
                matchedGoals.add(g)
                matchedBoxes.add(b)
                totalPath += p

        notAssignedBox = set()
        for b in self.boxes:
            if b not in matchedBoxes:
                notAssignedBox.add(b)

        for g in self.goals:
            if g not in matchedGoals:
                b = self.closestGoal(g, notAssignedBox)
                notAssignedBox.remove(b[0])
                totalPath += b[1]
        return totalPath


'''
Search algorithm
Blind (Breadth-First search)        = Search(initState, Queue())
Heuristic (A* or Best-First search) = Search(initState, PriorityQueue())
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

'''
Help run algorithms
    heuristic           # True: heuristic search | False: Blind search (Breath first search)
    detectDeadlock      # Detect deadlock?
    optimalHeuristic    # True: get optimal heuristic solution (i.e. A* Search) | False: Best-First Search
    greedy              # True: greedy assigment | False: Closest assignment 
'''
def helpRun(filename, detectDeadlock = True, heuristic = True, optimalHeuristic = True, greedy = True):
    # Set mode
    SetState.setMode(greedy=greedy, optimal=optimalHeuristic, detectDeadlock=detectDeadlock)
    queue = PriorityQueue() if heuristic else Queue()
    # Init state
    initState = SetState()
    initState.initMap(filename)
    # Run algorithm
    start = time()
    goalState, nodeVisited, nodeCreated = Search(initState, queue)
    end = time()
    runTime = round(end - start, 4)
    # Return result
    return goalState.route, runTime, nodeVisited, nodeCreated

    
########################################################################################################
#####################################  STOP HERE !!!  ##################################################
########################################################################################################
#############################  Behind is the UI implementation  ########################################
########################################################################################################

class MatrixState:
    '''
    Use to print solution
    Don't care me!!
    '''
    def __init__(self, filename):
        setState = SetState()
        self.map = setState.initMap(filename)
        for iRow in range(len(self.map)):
            self.map[iRow] = list(self.map[iRow])
            if PLAYER in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(PLAYER))
            if POG in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(POG))
            if POD in self.map[iRow]:
                self.player = Position(iRow, self.map[iRow].index(POD))
         
    def __repr__(self):
        return '\n'.join([' '.join(row) for row in self.map])

    def push(self, direction: Position):
        box = self.player + direction
        nextOfBox = box + direction
        if self.map[nextOfBox.row][nextOfBox.col] == GOAL:
            self.map[nextOfBox.row][nextOfBox.col] = BOG
        else:
            self.map[nextOfBox.row][nextOfBox.col] = BOX
        
        if self.map[box.row][box.col] == BOG:
            self.map[box.row][box.col] = GOAL
        else:
            self.map[box.row][box.col] = FLOOR

    def move(self, direction: Position):
        next = self.player + direction
        if self.map[next.row][next.col] in {BOX, BOG}:
            self.push(direction)

        self.map[next.row][next.col] = POG if self.map[next.row][next.col] == GOAL \
                                        else (POD if self.map[next.row][next.col] == DEADLOCK else PLAYER)
        self.map[self.player.row][self.player.col] = GOAL if self.map[self.player.row][self.player.col] == POG \
                                        else (DEADLOCK if self.map[self.player.row][self.player.col] == POD else FLOOR)

        self.player += direction

# declare final variables
WIDTH = 860
HEIGHT = 640

WHITE = (255,255,255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)
GREY = (211, 211, 211)
GREEN = (0, 255, 0)

# create window
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('SOKOBAN')
win.fill(BLACK)
newfilename = 'map/map.txt'

def buttonsImg(x, y, Img):
    w, h = 200, 200
    Img = pygame.transform.scale(Img, (w,h))
    win.blit(Img, (x,y))

def buttons(x, y, w, h, color, msg, size):
    fontb = pygame.font.SysFont("arial", size)
    text = fontb.render(msg, True, BLACK)
    pygame.draw.rect(win, color,(x, y, w, h))
    # draw button text
    textplace = text.get_rect(center=(x + w/2, y + h/2))
    win.blit(text, textplace)

def DrawState(state: MatrixState, dectectLock):
    blockSize = 50
    for row in range(len(state.map)):
        for col in range(len(state.map[row])):
            if (dectectLock == False):
                if (state.map[row][col] == '+'):
                    state.map[row][col] = 'p'
                if (state.map[row][col] == '-'):
                    state.map[row][col] = ' '
            if (state.map[row][col] == '#'):
                icon = pygame.image.load('img\icon_wall.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == 'p'):
                icon = pygame.image.load('img\icon_player.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == 'O'):
                icon = pygame.image.load('img\icon_box.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == 'X'):
                icon = pygame.image.load('img\icon_goal.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == '='):
                icon = pygame.image.load('img\icon_BOG.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == 'P'):
                icon = pygame.image.load('img\icon_POG.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
            elif (state.map[row][col] == ' '):
                pygame.draw.rect(win, BLACK, (col*blockSize, row*blockSize, blockSize, blockSize))
            elif (state.map[row][col] == '-'):
                icon = pygame.image.load('img\deadlock.jpg')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
            elif (state.map[row][col] == '+'):
                icon = pygame.image.load('img\POD.png')
                icon = pygame.transform.scale(icon, (blockSize, blockSize))
                win.blit(icon, (col*blockSize, row*blockSize))
                pass
    pass

def draw(initialState: MatrixState, route, detectLock):
    state = deepcopy(initialState)
    for move in route:
        pygame.event.get()
        sleep(0.15)
        pygame.draw.rect(win, BLACK, (0, 0, 650,500))
        state.move(move)
        DrawState(state, detectLock)
        pygame.display.update()

class Display:

    def Display1(self):
        buttons(230, 50, 400, 100, RED, "Let's Begin", 60)
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
            mouse = pygame.mouse.get_pos()
            click = pygame.mouse.get_pressed()
            if 230 < mouse[0] < 230+400 and 50 < mouse[1] < 50+100:
                buttons(230, 50, 400, 100, GREEN, "Let's Begin", 60)
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    self.Display2_0()
            else:
                buttons(230, 50, 400, 100, RED, "Let's Begin", 60)
            pygame.display.flip()
            clock.tick(15)
        
    def Display2_0(self):
        global newfilename
        win.fill(BLACK)
        buttons(WIDTH-60, 300, 60, 40, GREY, "Next >", 15)
        map0 = pygame.image.load('img\mini1.png')
        buttonsImg(80, 100, map0)
        map1 = pygame.image.load('img\mini2.png')
        buttonsImg(320, 100, map1)
        map2 = pygame.image.load('img\mini3.png')
        buttonsImg(560, 100, map2)
        map3 = pygame.image.load('img\mini4.png')
        buttonsImg(80, 340, map3)
        map4 = pygame.image.load('img\mini5.png')
        buttonsImg(320, 340, map4)
        map5 = pygame.image.load('img\mini6.png')
        buttonsImg(560, 340, map5)

        while True:
            font1 = pygame.font.SysFont("arial", 40)
            select = font1.render("Mini Cosmos", True, GREY, BLACK)
            wRect = select.get_rect(center=(WIDTH / 2, 40))
            win.blit(select, wRect)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            mouse = pygame.mouse.get_pos()
            click = pygame.mouse.get_pressed()
            if WIDTH-60 < mouse[0] < WIDTH and 300 < mouse[1] < 300+40:
                buttons(WIDTH-60, 300, 60, 40, GREEN, "Next >", 15)
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    self.Display2_1()
            elif 80 < mouse[0] < 80+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini01.txt'
                    self.Display3()
            elif 320 < mouse[0] < 320+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini04.txt'
                    self.Display3()
            elif 560 < mouse[0] < 560+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini07.txt'
                    self.Display3()
            elif 80 < mouse[0] < 80+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini10.txt'
                    self.Display3()
            elif 320 < mouse[0] < 320+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini13.txt'
                    self.Display3()
            elif 560 < mouse[0] < 560+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini16.txt'
                    self.Display3()
            else:
                buttons(WIDTH-60, 300, 60, 40, GREY, "Next >", 15)
            pygame.display.flip()
            clock.tick(15)

    def Display2_1(self):
        global newfilename
        win.fill(BLACK)
        buttons(0, 300, 60, 40, GREY, "< Previous", 15)
        map0 = pygame.image.load('img\micro1.png')
        buttonsImg(80, 100, map0)
        map1 = pygame.image.load('img\micro2.png')
        buttonsImg(320, 100, map1)
        map2 = pygame.image.load('img\micro3.png')
        buttonsImg(560, 100, map2)
        map3 = pygame.image.load('img\micro4.png')
        buttonsImg(80, 340, map3)
        map4 = pygame.image.load('img\micro5.png')
        buttonsImg(320, 340, map4)
        map5 = pygame.image.load('img\micro6.png')
        buttonsImg(560, 340, map5)

        while True:
            font1 = pygame.font.SysFont("arial", 40)
            select = font1.render("Micro Cosmos", True, GREY, BLACK)
            wRect = select.get_rect(center=(WIDTH / 2, 40))
            win.blit(select, wRect)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            mouse = pygame.mouse.get_pos()
            click = pygame.mouse.get_pressed()
            if 0 < mouse[0] < 0+60 and 300 < mouse[1] < 300+40:
                buttons(0, 300, 60, 40, GREEN, "< Previous", 15)
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    self.Display2_0()
            elif 80 < mouse[0] < 80+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/micro/micro03.txt'
                    self.Display3()
            elif 320 < mouse[0] < 320+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/micro/micro07.txt'
                    self.Display3()
            elif 560 < mouse[0] < 560+200 and 100 < mouse[1] < 100+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/mini/mini40.txt'
                    self.Display3()
            elif 80 < mouse[0] < 80+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/micro/micro28.txt'
                    self.Display3()
            elif 320 < mouse[0] < 320+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/micro/micro32.txt'
                    self.Display3()
            elif 560 < mouse[0] < 560+200 and 340 < mouse[1] < 340+200:
                if click != None and click[0] == 1:
                    pygame.time.wait(100)
                    newfilename = 'map/micro/micro40.txt'
                    self.Display3()
            else:
                buttons(0, 300, 60, 40, GREY, "< Previous", 15)
            pygame.display.flip() 
            clock.tick(15)

    def Display3(self):
        win.fill(BLACK)
        pygame.draw.line(win, WHITE, (0,500), (WIDTH,500), 4)
        pygame.draw.line(win, WHITE, (650,0), (650,500), 4)
        buttons(40, 520, 80, 40, RED, "BLI", 20)        # BLIND BUTTON
        buttons(40, 580, 80, 40, RED, "HEU", 20)        # HEURISTIC BUTTON
        buttons(200, 520, 80, 40, RED, "BEST-FS", 20)   # BEST-FS BUTTON
        buttons(200, 580, 80, 40, RED, "A*", 20)        # A* BUTTON
        buttons(360, 520, 80, 40, RED, "DeadLock", 20)  # DEADLOCK BUTTON
        buttons(360, 580, 80, 40, RED, "No", 20)        # NODEADLOCK BUTTON
        buttons(520, 520, 80, 40, RED, "Closest", 20)   # CLOSEST BUTTON
        buttons(520, 580, 80, 40, RED, "Greedy", 20)    # GREEDY BUTTON

        buttons(730-50, 570-50, 100, 100, GREY, "START", 20)     # START BUTTON
        running = True
        ready = [False, False, False, False]
        ##################
        detectDeadlock = True
        heuristic = True
        optimalHeuristic = False
        greedy = True

        matrixState = MatrixState(newfilename)
        DrawState(deepcopy(matrixState), False)
        ##################
        ran = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
            mouse = pygame.mouse.get_pos()
            click = pygame.mouse.get_pressed()
            if (not ready[0]):
                if 40 < mouse[0] < 40+80 and 520 < mouse[1] < 520+40:
                    buttons(40, 520, 80, 40, GREEN, "BLI", 20)        # BLIND BUTTON
                    if click != None and click[0] == 1:
                        ready[0] = True
                        heuristic = False
                        pygame.time.wait(1)
                elif 40 < mouse[0] < 40+80 and 580 < mouse[1] < 580+40:
                    buttons(40, 580, 80, 40, GREEN, "HEU", 20)       # HEURISTIC BUTTON
                    if click != None and click[0] == 1:
                        ready[0] = True
                        heuristic = True
                        pygame.time.wait(1)
                else:
                    buttons(40, 520, 80, 40, RED, "BLI", 20)        # BLIND BUTTON
                    buttons(40, 580, 80, 40, RED, "HEU", 20)       # HEURISTIC BUTTON      # HEURISTIC BUTTON
            if (not ready[1]):
                if 360 < mouse[0] < 360+80 and 520 < mouse[1] < 520+40:
                    buttons(360, 520, 80, 40, GREEN, "DeadLock", 20)  # DEADLOCK BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        detectDeadlock = True
                        ready[1] = True
                elif 360 < mouse[0] < 360+80 and 580 < mouse[1] < 580+40:
                    buttons(360, 580, 80, 40, GREEN, "No", 20)        # NODEADLOCK BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        detectDeadlock = False
                        ready[1] = True
                else:
                    buttons(360, 520, 80, 40, RED, "DeadLock", 20)  # DEADLOCK BUTTON
                    buttons(360, 580, 80, 40, RED, "No", 20)        # NODEADLOCK BUTTON
            if (not ready[2]):
                if 520 < mouse[0] < 520+80 and 520 < mouse[1] < 520+40:
                    buttons(520, 520, 80, 40, GREEN, "Closest", 20)   # CLOSEST BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        greedy = False
                        ready[2] = True
                elif 520 < mouse[0] < 520+80 and 580 < mouse[1] < 580+40:
                    buttons(520, 580, 80, 40, GREEN, "Greedy", 20)    # GREEDY BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        greedy = True
                        ready[2] = True
                else:
                    buttons(520, 520, 80, 40, RED, "Closest", 20)   # CLOSEST BUTTON
                    buttons(520, 580, 80, 40, RED, "Greedy", 20)    # GREEDY BUTTON
            if (not ready[3]):
                if 200 < mouse[0] < 200+80 and 520 < mouse[1] < 520+40:
                    buttons(200, 520, 80, 40, GREEN, "BEST-FS", 20)   # BEST-FS BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        optimalHeuristic = False
                        ready[3] = True
                elif 200 < mouse[0] < 200+80 and 580 < mouse[1] < 580+40:
                    buttons(200, 580, 80, 40, GREEN, "A*", 20)    # A* BUTTON
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        optimalHeuristic = True
                        ready[3] = True
                else:
                    buttons(200, 520, 80, 40, RED, "BEST-FS", 20)   # BEST-FS BUTTON
                    buttons(200, 580, 80, 40, RED, "A*", 20)    # A* BUTTON
            if (ready[0] == True and ready[1] == True and ready[2] == True and ready[3] == True):
                buttons(730-50, 570-50, 100, 100, RED, "START", 20)
                if 730-50 < mouse[0] < 730-50+100 and 570-50 < mouse[1] < 570-50+100:
                    buttons(730-50, 570-50, 100, 100, GREEN, "START", 20)
                    if click != None and click[0] == 1:
                        pygame.time.wait(1)
                        
                        font1 = pygame.font.SysFont("arial", 20)
                        t1 = font1.render("Running...", True, GREY, BLACK)
                        win.blit(t1, (680, 70))
                        pygame.display.flip()
                        
                        route, runTime, nodeVisited, nodeCreated = helpRun(newfilename, detectDeadlock, heuristic, optimalHeuristic, greedy)
                        #pygame.draw.rect(win, BLACK, (680, 70, 50,50))
                        t1 = font1.render("Duration: " + str(runTime)[0:6] + " (s)", True, GREY, BLACK)
                        win.blit(t1, (680, 70))
                        t1 = font1.render("Step: " + str(len(route)), True, GREY, BLACK)
                        win.blit(t1, (680, 120))
                        t1 = font1.render("Node visited: " + str(nodeVisited), True, GREY, BLACK)
                        win.blit(t1, (680, 170))
                        t1 = font1.render("Node generated: " + str(nodeCreated), True, GREY, BLACK)
                        win.blit(t1, (680, 220))

                        buttons(680, 350, 150, 30, RED, "RUN", 20)
                        buttons(680, 300, 150, 30, RED, "Select another map", 20)
                        buttons(730-50, 570-50, 100, 100, RED, "AGAIN", 20)
                        while True:
                            for event in pygame.event.get():
                                if event.type == pygame.QUIT:
                                    pygame.quit()
                                    quit()
                            mouse = pygame.mouse.get_pos()
                            click = pygame.mouse.get_pressed()
                            buttons(680, 350, 150, 30, RED, "RUN", 20)
                            buttons(680, 300, 150, 30, RED, "Select another map", 20)
                            buttons(730-50, 570-50, 100, 100, RED, "AGAIN", 20)
                            if 680 < mouse[0] < 680+150 and 300 < mouse[1] < 300+30:
                                buttons(680, 300, 150, 30, GREEN, "Select another map", 20)
                                if click != None and click[0] == 1:
                                    pygame.time.wait(100)
                                    self.Display2_0()
                            elif 680 < mouse[0] < 680+150 and 350 < mouse[1] < 350+30:
                                buttons(680, 350, 150, 30, GREEN, "RUN", 20)
                                if click != None and click[0] == 1:
                                    draw(matrixState, route, detectDeadlock)
                            elif 730-50 < mouse[0] < 730-50+100 and 570-50 < mouse[1] < 570-50+100:
                                buttons(730-50, 570-50, 100, 100, GREEN, "AGAIN", 20)
                                if click != None and click[0] == 1:
                                    self.Display3()
                                    pygame.time.wait(100)
                            else:
                                buttons(680, 350, 150, 30, RED, "RUN", 20)
                                buttons(680, 300, 150, 30, RED, "Select another map", 20)
                                buttons(730-50, 570-50, 100, 100, RED, "AGAIN", 20)
                            pygame.display.flip()
                            clock.tick(15)

                else:
                    buttons(730-50, 570-50, 100, 100, RED, "START", 20)
            pygame.display.flip()
            clock.tick(15)


if __name__ == '__main__':
    pygame.init()
    clock = pygame.time.Clock()
    displayWindow = Display()
    displayWindow.Display1()
