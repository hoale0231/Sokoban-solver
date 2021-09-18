from queue import Queue
from collections import deque
from copy import copy, deepcopy
import time
import os
# initial 2-D matrix
# 'g' = goal, 'b' = box, 'm' = man, move man to push box to goal


class Matrix:

    # print matrix
    # def print_matrix(T):
    #     for r in T:
    #         for c in r:
    #             print(c,end = " ")
    #     print()

    def __init__(self):
        self.T = [[0 for x in range(8)] for y in range(8)]

    def initMatrix(self):
        for i in range(8):
            self.T[i] = [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']
        for i in range(8):
            self.T[0][i] = '#'
            self.T[7][i] = '#'
            self.T[i][0] = '#'
            self.T[i][7] = '#'
        self.T[2][3] = 'g'
        self.T[3][6] = 'g'
        self.T[4][2] = 'g'
        self.T[6][4] = 'g'
        self.T[3][3] = 'b'
        self.T[3][5] = 'b'
        self.T[4][3] = 'b'
        self.T[5][4] = 'b'
        self.T[4][4] = 'm'

    # move down
    def move_down(self, flag):
        row = 0
        for i in self.T:
            col = 0
            for j in i:
                if j == 'm':
                    row = row + 1
                    if self.T[row][col] == '#' or (self.T[row][col] == 'b' and self.T[row + 1][col] == '#'):
                        flag = False
                        return
                    elif self.T[row][col] == 'b':
                        self.T[row+1][col] = 'b'
                    # incase it's a goal 'g' ?
                    self.T[row][col] = 'm'
                    self.T[row-1][col] = ' '
                    flag = True
                    return
                col = col + 1
            row = row + 1
    # move up

    def move_up(self, flag):
        row = 0
        for i in self.T:
            col = 0
            for j in i:
                if j == 'm':
                    row = row - 1
                    if self.T[row][col] == '#' or (self.T[row][col] == 'b' and self.T[row - 1][col] == '#'):
                        flag = False
                        return
                    elif self.T[row][col] == 'b':
                        self.T[row-1][col] = 'b'
                    # incase it's a goal 'g' ?
                    self.T[row][col] = 'm'
                    self.T[row+1][col] = ' '
                    flag = True
                    return
                col = col + 1
            row = row + 1
    # move right

    def move_right(self, flag):
        row = 0
        for i in self.T:
            col = 0
            for j in i:
                if j == 'm':
                    col = col + 1
                    if self.T[row][col] == '#' or (self.T[row][col] == 'b' and self.T[row][col + 1] == '#'):
                        flag = False
                        return
                    elif self.T[row][col] == 'b':
                        self.T[row][col + 1] = 'b'
                    # incase it's a goal 'g' ?
                    self.T[row][col] = 'm'
                    self.T[row][col-1] = ' '
                    flag = True
                    return
                col = col + 1
            row = row + 1
    # move left

    def move_left(self, flag):
        row = 0
        for i in self.T:
            col = 0
            for j in i:
                if j == 'm':
                    col = col - 1
                    if self.T[row][col] == '#' or (self.T[row][col] == 'b' and self.T[row][col - 1] == '#'):
                        flag = False
                        return
                    elif self.T[row][col] == 'b':
                        self.T[row][col - 1] = 'b'
                    # incase it's a goal 'g' ?
                    self.T[row][col] = 'm'
                    self.T[row][col + 1] = ' '
                    flag = True
                    return
                col = col + 1
            row = row + 1

    # print path
    def show_path(self, path):
        for p in path:
            time.sleep(0.5)
            os.system("cls")
            for r in self.T:
                for c in r:
                    print(c, end=" ")
                print()
            if p == 'UP':
                self.move_up(True)
            if p == 'DOWN':
                self.move_down(True)
            if p == 'RIGHT':
                self.move_right(True)
            if p == 'LEFT':
                self.move_left(True)


    # AI
savePath = ()


def BFS(T):
    count_step = 0
    isSolvable = False
    q = deque()
    visited = []
    q.append((T, savePath))
    visited.append(T)
    goalstate = []
    for i in range(len(T)):
        for j in range(len(T)):
            if T[i][j] == 'g':
                goalstate.append((i, j))
    while (len(q) > 0):
        # for i in range(10):
        # check if current status could reach goal
        first_ele = q.popleft()

        goal = False
        boxstate = []
        for i in first_ele[0]:
            for j in i:
                if j == 'g':
                    goal = True
        if goal == False:
            isSolvable = True
            print(count_step)
            U = Matrix()
            U.initMatrix()
            U.show_path(first_ele[1])
            return
        U = Matrix()
        U.T = deepcopy(first_ele[0])

        flag1 = True

        U.move_up(flag1)
        if U.T not in visited:
            addtuple = ("UP",)
            path = first_ele[1] + addtuple
            if flag1 == True:
                q.append((U.T, path))
                count_step += 1
                visited.append(U.T)

        U.T = deepcopy(first_ele[0])
        flag1 = True
        U.move_down(flag1)
        if U.T not in visited:
            addtuple = ("DOWN",)
            path = first_ele[1] + addtuple
            if flag1 == True:
                q.append((U.T, path))
                count_step += 1
                visited.append(U.T)

        U.T = deepcopy(first_ele[0])
        flag1 = True
        U.move_left(flag1)
        if U.T not in visited:
            addtuple = ("LEFT",)
            path = first_ele[1] + addtuple
            if flag1 == True:
                q.append((U.T, path))
                count_step += 1
                visited.append(U.T)

        U.T = deepcopy(first_ele[0])
        flag1 = True
        U.move_right(flag1)
        if U.T not in visited:
            addtuple = ("RIGHT",)
            path = first_ele[1] + addtuple
            if flag1 == True:
                q.append((U.T, path))
                count_step += 1
                visited.append(U.T)

    if isSolvable == False:
        print(count_step)
        print("No solution")


initState = Matrix()
initState.initMatrix()

for r in initState.T:
    for c in r:
        print(c, end=" ")
    print()

BFS(initState.T)
