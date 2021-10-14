from queue import Queue, PriorityQueue
from copy import deepcopy
from time import time, sleep
from sys import argv
from os import system, listdir
from multiprocessing import Process, Manager
from pandas import DataFrame
from sokoban import MatrixState, SetState, Search

def printSolution(initState: MatrixState, route):
    input("Press Enter to continue...")
    state = deepcopy(initState)
    for move in route:
        sleep(1)
        state.move(move)
        system('cls')
        print(state)

def helpRunLevel(filename, dataSet, detectDeadlock = True, heuristic = True, optimalHeuristic = True, greedy = True):
    SetState.setMode(greedy=greedy, optimal=optimalHeuristic, detectDeadlock=detectDeadlock)
    queue = PriorityQueue() if heuristic else Queue()
    initState = SetState()
    initState.initMap(filename)
    
    start = time()
    goalState, nodeVisited, nodeCreated = Search(initState, queue)
    end = time()
    runTime = round(end - start, 4)
    
    dataSet['Run time'] = runTime
    dataSet['Steps'] = len(goalState.route)
    dataSet['Visited'] = nodeVisited
    dataSet['Generated'] = nodeCreated
    print(filename + " done.")

def runAllLevel(folderName, detectDeadlock = True, heuristic = True, optimalHeuristic = True, greedy = True):
    files = listdir(folderName)
    manager = Manager()
    returnValue = manager.dict()
    size = len(files)
    dataSet = {
        'Run time': [None]*size,
        'Steps':  [None]*size,
        'Visited':  [None]*size,
        'Generated':  [None]*size
    }

    for i in range(size):
        P = Process(target=helpRunLevel, args=[folderName+'/'+files[i], returnValue, detectDeadlock, heuristic, optimalHeuristic, greedy])
        P.start()
        P.join(timeout=300)
        if P.is_alive():
            dataSet['Run time'][i] = 'Time out'
            P.terminate()
        else:
            dataSet['Run time'][i] = returnValue['Run time']
            dataSet['Steps'][i] = returnValue['Steps']
            dataSet['Visited'][i] = returnValue['Visited']
            dataSet['Generated'][i] = returnValue['Generated']
    
    df = DataFrame(dataSet)
    df.to_excel(folderName+'Agreedy'+'.xlsx', sheet_name='sheet1', index=False)

if __name__ == '__main__':
    runAllLevel('map/mini')
    #runAllLevel('map/micro')