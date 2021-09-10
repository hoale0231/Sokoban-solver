class State:
    def __init__(self, filename):
        f = open(filename)
        self.map = f.read().split('\n')
        for iRow in range(len(self.map)):
            if self.map[iRow].find('p') == -1:
                continue
            else:
                self.player = [iRow, self.map[iRow].find('p')]
                break

    def __hash__(self):
        return hash(''.join(self.map))

    def __repr__(self):
        return '\n'.join(self.map)

    def getPosition(self):
        return self.player

    def moveUp():
        pass

    def moveDown():
        pass

    def moveRight():
        pass

    def moveLeft():
        pass


filename = 'map.txt'

m = State(filename)

print(m)