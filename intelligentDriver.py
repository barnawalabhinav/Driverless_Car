'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import heapq
import itertools
import random
import math
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError

# Class: Graph
# -------------
# Utility class


class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially.


class IntelligentDriver(Junior):

    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout
        self.costFactor = 1000
        self.paddedBlocks = []
        self.worldGraph = self.createWorldGraph()
        self.waitingSince = 0
        self.maxWait = 0
        # a list of single tile locations corresponding to each checkpoint
        self.checkPoints = self.layout.getCheckPoints()
        self.transProb = util.loadTransProb()
        self.carLocations = []

    def getNodeIdentifier(self, node):
        (x, y) = node
        return self.layout.getBeliefCols()*x + y

    def getContours(self, pad_block):
        x, y = pad_block[0], pad_block[1]
        adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
        contour = []
        for tile in adjNodes:
            if tile not in self.worldGraph.nodes:
                contour.append(tile)
        return contour

    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        # each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(
            range(numRows), range(numCols))]

        # EDGES #
        # Adjacency Matrix #
        edges = [[0 for _ in range(len(nodes))] for _ in range(len(nodes))]

        # EDGES #
        # We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        # avoid the tiles representing walls or blocks
        # YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        # FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        # Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2]
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            blockWidth = col2-col1
            blockHeight = row2-row1
            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)
            Erow1, Ecol1, Erow2, Ecol2 = row1-1, col1-1, row2+1, col2+1
            for r in range(row1, row2):
                self.paddedBlocks.append((r, Ecol1))
                self.paddedBlocks.append((r, Ecol2-1))
            for c in range(col1, col2):
                self.paddedBlocks.append((Erow1, c))
                self.paddedBlocks.append((Erow2-1, c))

        for r in range(0, self.layout.getBeliefRows()):
            self.paddedBlocks.append((r, 0))
            self.paddedBlocks.append((r, self.layout.getBeliefCols()-1))
        for c in range(0, self.layout.getBeliefCols()):
            self.paddedBlocks.append((0, c))
            self.paddedBlocks.append((self.layout.getBeliefRows()-1, c))

        # Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            # only keep allowed (within boundary) adjacent nodes
            for tile in adjNodes:
                if tile[0] >= 0 and tile[1] >= 0 and tile[0] < numRows and tile[1] < numCols:
                    if tile not in blockTiles:
                        if tile in self.paddedBlocks:
                            edges[self.getNodeIdentifier(node)][self.getNodeIdentifier(
                                tile)] = self.costFactor/1000
                        else:
                            edges[self.getNodeIdentifier(
                                node)][self.getNodeIdentifier(tile)] = 1

        return Graph(nodes, edges)

    def modifyWorldGraph(self, beliefOfOtherCars: list, checkPoint, parkedCars):
        carsLikelihood = [[0 for _ in range(self.layout.getBeliefCols())] for _ in range(
            self.layout.getBeliefRows())]
        for carNum in range(len(beliefOfOtherCars)):
            grid = beliefOfOtherCars[carNum].grid
            for row in range(len(grid)):
                for col in range(len(grid[row])):
                    node = (row, col)
                    carsLikelihood[row][col] += grid[row][col]
                    node = (row, col)
                    rows = [row, row-1, row+1, row+2, row-2]
                    cols = [col, col-1, col+1, col+2, col-2]
                    for r in rows:
                        for c in cols:
                            if r >= 0 and r < len(grid) and c >= 0 and c < len(grid[row]):
                                if parkedCars[carNum]:
                                    if r == row+2 or r == row-2 or c == col+2 or c == col-2:
                                        carsLikelihood[r][c] += grid[row][col]/5
                                    else:
                                        carsLikelihood[r][c] += grid[row][col]/2
                                else:
                                    if r == row+1 or r == row-1 or c == col+1 or c == col-1 or (r == row and c == col):
                                        carsLikelihood[r][c] += grid[row][col]/5

        total = 0
        for row in range(len(carsLikelihood)):
            for col in range(len(carsLikelihood[row])):
                total += carsLikelihood[row][col]
        for row in range(len(carsLikelihood)):
            for col in range(len(carsLikelihood[row])):
                carsLikelihood[row][col] /= total

        for node in self.worldGraph.nodes:
            (x, y) = node
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            for (row, col) in adjNodes:
                if (row, col) in checkPoint:
                    self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier((row, col))] = 1 + 10*self.costFactor*(carsLikelihood[row][col] + carsLikelihood[node[0]][node[1]])/2
                if row >= 0 and col >= 0 and row < self.layout.getBeliefRows() and col < self.layout.getBeliefCols():
                    self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier((row, col))] = 1 + self.costFactor*carsLikelihood[row][col]
        return carsLikelihood

    def getShortestPathUsingDijkstra(self, start: tuple, end: tuple, beliefOfOtherCars: list, parkedCars: list):
        # initialize
        likelihood = self.modifyWorldGraph(beliefOfOtherCars, end, parkedCars)
        visited = {}
        distance = {}
        prev = {}
        for node in self.worldGraph.nodes:
            distance[node] = float('inf')
            prev[node] = None
            visited[node] = False
        distance[start] = 0
        pathFound = False
        priorityQueue = [(0, start)]

        # main loop
        while (False in visited.values()):
            minDistance, minNode = heapq.heappop(priorityQueue)
            for node in self.worldGraph.nodes:
                if not visited[node] and distance[node] < minDistance:
                    minDistance = distance[node]
                    minNode = node
            visited[minNode] = True
            if minNode == end:
                pathFound = True
                break

            # update distance
            for ngbr in self.worldGraph.nodes:
                edgeCost = self.worldGraph.edges[self.getNodeIdentifier(
                    minNode)][self.getNodeIdentifier(ngbr)]
                if edgeCost > 0 and not visited[ngbr]:
                    if minDistance + edgeCost < distance[ngbr]:
                        distance[ngbr] = minDistance + edgeCost
                        heapq.heappush(priorityQueue, (distance[ngbr], ngbr))
                        prev[ngbr] = minNode

        # find the path
        if (pathFound):
            node = end
            while prev[node] != start:
                node = prev[node]
            x_offset = 0
            y_offset = 0

            temp_vectorToGoal = (util.colToX(
                node[1]), util.rowToY(node[0])) - self.pos
            temp_wheelAngle = -temp_vectorToGoal.get_angle_between(self.dir)

            if node in self.paddedBlocks:
                blockedAreas = self.getContours(node)
                for block in blockedAreas:
                    if abs(temp_wheelAngle) > 10 and (node[0] == block[0]):
                        x_offset = (node[1] - block[1])*Car.LENGTH*0.5
                    elif abs(temp_wheelAngle) > 10 and (node[1] == block[1]):
                        y_offset = (node[0] - block[0])*Car.LENGTH*0.5

            self.maxWait = 0
            carParked = False
            for (r_Car, c_Car) in self.carLocations:
                for i in range(-1, 2, 1):
                    for j in range(-1, 2, 1):
                        if (r_Car + i, c_Car + j) == node:
                            carParked = True
                            self.maxWait = likelihood[node[0]][node[1]]*100
                            break
                    if carParked:
                        break
                if carParked:
                    break
            
            if not carParked:
                if likelihood[node[0]][node[1]] > 0.3:
                    self.maxWait = float('inf')
                else:
                    self.maxWait = likelihood[node[0]][node[1]]*500
            else:
                self.maxWait = likelihood[node[0]][node[1]]*300

            if abs(temp_wheelAngle) > 10 and (start in self.checkPoints or start in self.paddedBlocks or node in self.checkPoints or node in self.paddedBlocks):
                self.maxWait = max(self.maxWait, 5)

            if self.maxWait > 0:
                return node, False, (x_offset, y_offset)
            return node, True, (x_offset, y_offset)
        else:
            return start, False, (x_offset, y_offset)

    def updateBeliefOfOtherCars(self, beliefOfOtherCars: list, parkedCars: list):
        for carId in range(len(beliefOfOtherCars)):
            belief = beliefOfOtherCars[carId]
            if not parkedCars[carId]:
                newBelief = [[0 for col in range(self.layout.getBeliefCols())] for row in range(
                    self.layout.getBeliefRows())]
                rows = belief.numRows
                cols = belief.numCols
                init_weights = [belief.grid[i][j]
                                for i in range(rows) for j in range(cols)]
                init_particles = [
                    i*cols + j for i in range(rows) for j in range(cols)]
                particles = random.choices(
                    init_particles, init_weights, k=len(init_particles))
                i = 0
                while (i < len(particles)):
                    particle = particles[i]
                    y = particle // cols
                    x = particle % cols
                    moving_prob = [self.transProb[((y, x), (y-1, x-1))] if ((y, x), (y-1, x-1)) in self.transProb else 0,
                                   self.transProb[(
                                       (y, x), (y, x-1))] if ((y, x), (y, x-1)) in self.transProb else 0,
                                   self.transProb[(
                                       (y, x), (y+1, x-1))] if ((y, x), (y+1, x-1)) in self.transProb else 0,
                                   self.transProb[(
                                       (y, x), (y-1, x))] if ((y, x), (y-1, x)) in self.transProb else 0,
                                   self.transProb[((y, x), (y, x))] if (
                        (y, x), (y, x)) in self.transProb else 0,
                        self.transProb[(
                                       (y, x), (y+1, x))] if ((y, x), (y+1, x)) in self.transProb else 0,
                        self.transProb[(
                                       (y, x), (y-1, x+1))] if ((y, x), (y-1, x+1)) in self.transProb else 0,
                        self.transProb[(
                                       (y, x), (y, x+1))] if ((y, x), (y, x+1)) in self.transProb else 0,
                        self.transProb[((y, x), (y+1, x+1))] if ((y, x), (y+1, x+1)) in self.transProb else 0]

                    total = sum(moving_prob)
                    if total == 0:
                        particles.remove(particle)
                    else:
                        moving_prob = [prob/total for prob in moving_prob]
                        particles[i] = random.choices([x-1+(y-1)*cols, x-1+y*cols, x-1+(y+1)*cols, x+(
                            y-1)*cols, x+y*cols, x+(y+1)*cols, x+1+(y-1)*cols, x+1+y*cols, x+1+(y+1)*cols], moving_prob, k=1)[0]
                        i += 1
                newTotal = 0
                for particle in particles:
                    r = particle // cols
                    c = particle % cols
                    newBelief[r][c] += 1
                    newTotal += 1
                for particle in particles:
                    r = particle // cols
                    c = particle % cols
                    belief.addProb(r, c, newBelief[r][c]/newTotal)
                belief.normalize()
            else:
                max_row = -1
                max_col = -1
                max_belief = 0
                for row in range(belief.numRows):
                    for col in range(belief.numCols):
                        if belief.grid[row][col] > max_belief:
                            max_row = row
                            max_col = col
                            max_belief = belief.grid[row][col]
                self.carLocations[carId] = (max_row, max_col)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!

        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''
        (curr_x, curr_y) = self.getPos()  # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        curr_row = util.yToRow(curr_y)
        curr_col = util.xToCol(curr_x)
        (goal_Row, goal_Col) = self.checkPoints[chkPtsSoFar]
        for _ in range(len(beliefOfOtherCars)):
            self.carLocations.append((-2, -2))
        self.updateBeliefOfOtherCars(beliefOfOtherCars, parkedCars)
        (next_row, next_col), moveForward, offset = self.getShortestPathUsingDijkstra(
            (curr_row, curr_col), (goal_Row, goal_Col), beliefOfOtherCars, parkedCars)

        goalPos = (util.colToX(next_col) +
                   offset[0], util.rowToY(next_row) + offset[1])  # next tile

        # BEGIN_YOUR_CODE
        if not moveForward:
            self.waitingSince += 1
            if self.waitingSince > self.maxWait:
                self.waitingSince = 0
                moveForward = True
        else:
            self.waitingSince = 0
        
        self.carLocations.clear()

        # END_YOUR_CODE
        return goalPos, moveForward

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return []

        goalPos, df = self.getNextGoalPos(
            beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions
