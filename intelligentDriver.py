'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools
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
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()

        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        

    def getNodeIdentifier(self, node):
        (x, y) = node
        return self.layout.getBeliefCols()*x + y
        
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
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]

        # EDGES #
        # Adjacency Matrix #
        edges = [[0 for _ in range(len(nodes))] for _ in range(len(nodes))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        markedBlocks = []
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
            for r in range(Erow1, Erow2):
                markedBlocks.append((r, Ecol1))
                markedBlocks.append((r, Ecol2-1))
            for c in range(Ecol1, Ecol2):
                markedBlocks.append((Erow1, c))
                markedBlocks.append((Erow2-1, c))

        # print(markedBlocks)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            # adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y), (x+1,y+1), (x-1,y+1), (x+1,y-1), (x-1,y-1)]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            # only keep allowed (within boundary) adjacent nodes
            # adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        if tile in markedBlocks:
                            edges[self.getNodeIdentifier(node)][self.getNodeIdentifier(tile)] = self.costFactor
                        else:
                            edges[self.getNodeIdentifier(node)][self.getNodeIdentifier(tile)] = 1
                        # adjacentNodes.append(tile)

            # for tile in adjacentNodes:
            #     edges.append((node, tile))
            #     edges.append((tile, node))
        return Graph(nodes, edges)

    def modifyWorldGraph(self, beliefOfOtherCars: list, checkPoint):
        # markedNodes = {}
        origLikelihood = [[0 for _ in range(self.layout.getBeliefCols())] for _ in range(self.layout.getBeliefRows())]
        for carNum in range(len(beliefOfOtherCars)):
            grid = beliefOfOtherCars[carNum].grid
            for row in range(len(grid)):
                for col in range(len(grid[row])):
                    node = (row, col)
                    origLikelihood[row][col] += grid[row][col]
        carsLikelihood = origLikelihood.copy()
        grid = beliefOfOtherCars[carNum].grid
        for row in range(len(origLikelihood)):
            for col in range(len(origLikelihood[row])):
                node = (row, col)
                rows = [row, row-1, row+1]
                cols = [col, col-1, col+1]
                # markedNodes[node] = False
                for r in rows:
                    for c in cols:
                        if r >= 0 and r < len(grid) and c >= 0 and c < len(grid[row]):
                            carsLikelihood[r][c] += grid[row][col]/9
                            # markedNodes[node] = True
        total = 0
        for row in range(len(carsLikelihood)):
            for col in range(len(carsLikelihood[row])):
                total += carsLikelihood[row][col]
        for row in range(len(carsLikelihood)):
            for col in range(len(carsLikelihood[row])):
                carsLikelihood[row][col] /= total
        
        for node in self.worldGraph.nodes:
            (x, y) = node
            # adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y), (x+1,y+1), (x-1,y+1), (x+1,y-1), (x-1,y-1)]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y)]
            for (row, col) in adjNodes:
                if (row, col) == checkPoint:
                    self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier((row, col))] = 1
                elif row >= 0 and col >= 0 and row < self.layout.getBeliefRows() and col < self.layout.getBeliefCols():
                    self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier((row, col))] = 1 + max(self.costFactor*carsLikelihood[row][col], self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier((row, col))])


    def getShortestPathUsingBFS(self, start: tuple, end: tuple, beliefOfOtherCars):
        self.modifyWorldGraph(beliefOfOtherCars, end)
        queue = []
        visited = {}
        prev = {}
        for node in self.worldGraph.nodes:
            visited[node] = False
            prev[node] = None
        queue.append(start)
        visited[start] = True
        iter = 0
        pathFound = False
        while queue:
            iter += 1
            # print("Paths Computed = ", iter)
            node = queue.pop(0)
            if node == end:
                # print(f"found path to {end}")
                print("Path Found")
                pathFound = True
                break
            for ngbr in self.worldGraph.nodes:
                if self.worldGraph.edges[self.getNodeIdentifier(node)][self.getNodeIdentifier(ngbr)] > 0 and not visited[ngbr]:
                    prev[ngbr] = node
                    visited[ngbr] = True
                    if ngbr == end:
                        # print(f"found path to {end}")
                        print("Path Found")
                        pathFound = True
                        break
                    queue.append(ngbr)
            if pathFound:
                break

        if pathFound:
            node = end
            while prev[node] != start:
                node = prev[node]
            return node, True
        else:
            print("Path not found")
            return start, False

    def getShortestPathUsingDijkstra(self, start: tuple, end: tuple, beliefOfOtherCars: list):

        # initialize
        self.modifyWorldGraph(beliefOfOtherCars, end)
        visited = {}
        distance = {}
        prev = {}
        for node in self.worldGraph.nodes:
            distance[node] = float('inf')
            prev[node] = None
            visited[node] = False
        distance[start] = 0
        pathFound = False
        # main loop
        while (False in visited.values()):
            # find the node with the smallest distance
            minDistance = float('inf')
            minNode = start
            # for node in visited:
            #     if distance[node] < minDistance:
            #         minDistance = distance[node]
            #         minNode = node
            # print(f'{minNode}')


            for node in self.worldGraph.nodes:
                if not visited[node] and distance[node] < minDistance:
                    minDistance = distance[node]
                    minNode = node
            # print(minNode)
            visited[minNode] = True
            if minNode == end:
                print("Path Found")
                pathFound = True
                break
            
            # visited.append(minNode)
            # update distance
            for ngbr in self.worldGraph.nodes:
                edgeCost = self.worldGraph.edges[self.getNodeIdentifier(minNode)][self.getNodeIdentifier(ngbr)]
                if edgeCost > 0 and not visited[ngbr]:
                    # if edgeCost > 50:
                    #     print(f"{edgeCost} between {minNode} and {ngbr}")
                    if distance[minNode] + edgeCost < distance[ngbr]:
                               distance[ngbr] = distance[minNode] + edgeCost
                               prev[ngbr] = minNode

        # find the path
        # path = []
        if(pathFound):
            node = end
            while prev[node] != start:
                # path.append(node)
                node = prev[node]
            # path.append(start)
            # path.reverse()
            return node, True
        else:
            print("Path not found")
            return start, False

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
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
        (curr_x, curr_y) = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        curr_row = util.yToRow(curr_y)
        curr_col = util.xToCol(curr_x)
        (goal_Row, goal_Col) = self.checkPoints[chkPtsSoFar]
        (next_row, next_col), moveForward = self.getShortestPathUsingDijkstra((curr_row, curr_col), (goal_Row, goal_Col), beliefOfOtherCars)
        # print("CHECKPOINT : ", (goal_Row, goal_Col))
        # print("CURR : ", (curr_row, curr_col))
        # print("TARGET : ", (next_row, next_col))
        # if next_row != curr_row and next_col != curr_col:
        #     if (curr_row, next_col) not in self.worldGraph.nodes:
        #         next_row = curr_row
        #     elif (next_row, curr_col) not in self.worldGraph.nodes:
        #         next_col = curr_col

        goalPos = (util.colToX(next_col), util.rowToY(next_row)) # next tile
        # goalPos = (util.colToX(12), util.rowToY(6)) # next tile
         
        # BEGIN_YOUR_CODE 

        # END_YOUR_CODE
        return goalPos, moveForward

    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]

        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions
    
    