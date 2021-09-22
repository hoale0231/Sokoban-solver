from sokoban import SetState, Search
from queue import PriorityQueue
from pandas import DataFrame
from time import time
from sys import argv

if __name__ == '__main__':
    filename = 'map.txt' if len(argv) != 2 else argv[1]
    name = filename.split('.')
    duration = []
    visited = []
    created = []
    step = []
    alpha = []
    i = 1
    m = [[]]
    for i in range(1, 10):
        m.append([0])
        for j in range(1, 10):
            initState = SetState(i, j)
            initState.initMap('map/'+filename)
            heuristic, nheur, cntheu = Search(initState, PriorityQueue())
            m[i].append(nheur)

    # while i < 1000:
    #     print(i)
    #     start = time()
    #     initState = SetState(50, i)
    #     initState.initMap('map/'+filename)
    #     heuristic, nheur, cntheu = Search(initState, PriorityQueue())
    #     end = time()
    #     duration.append(end - start)
    #     visited.append(nheur)
    #     created.append(cntheu)
    #     step.append(len(heuristic.route))
    #     alpha.append(i)
    #     i *= 2
        
    
    # df = DataFrame({'alpha': alpha ,'step': step, 'duration': duration, 'node visited': visited, 'node created:': created})
    name[1] = 'xlsx'
    df = DataFrame(m)
    df.to_excel('data/'+'.'.join(name), sheet_name='sheet1', index=False)