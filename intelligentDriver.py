'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import time
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
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        
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
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        blockTiles = []
        for block in blocks:
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y), (x+1,y+1), (x-1,y+1), (x+1,y-1), (x-1,y-1)]
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                edges.append((tile, node))
        return Graph(nodes, edges)

    def modifyWorldGraph(self, beliefOfOtherCars: list):
        markedNodes = {}
        for carNum in range(len(beliefOfOtherCars)):
            grid = beliefOfOtherCars[carNum].grid
            for row in range(len(grid)):
                for col in range(len(grid[row])):
                    node = (row, col)
                    if grid[row][col] > 0.25:
                        markedNodes[node] = True
                    else:
                        markedNodes[node] = False
        return markedNodes

    def getShortestPathUsingBFS(self, start: tuple, end: tuple, beliefOfOtherCars):
        markedNodes = self.modifyWorldGraph(beliefOfOtherCars)
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
            for adjacent in self.worldGraph.edges:
                if adjacent[0] == node and (adjacent[1] == end or (not visited[adjacent[1]] and not markedNodes[adjacent[1]])):
                    prev[adjacent[1]] = node
                    visited[adjacent[1]] = True
                    queue.append(adjacent[1])

        if pathFound:
            path = []
            node = end
            while node != start:
                path.append(node)
                node = prev[node]
            path.append(start)
            path.reverse()
            return path[1]
        else:
            return None

    def getShortestPathUsingDijkstra(self, start: tuple, end: tuple, beliefOfOtherCars: list):
        # initialize
        markedNodes = self.modifyWorldGraph(beliefOfOtherCars)
        visited = {}
        distance = {}
        prev = {}
        for node in self.worldGraph.nodes:
            distance[node] = float('inf')
            visited[node] = False
            prev[node] = None
        distance[start] = 0

        # main loop
        while (False in visited.values()):
            # find the node with the smallest distance
            minDistance = float('inf')
            for node in self.worldGraph.nodes:
                if not visited[node] and distance[node] < minDistance:
                    minDistance = distance[node]
                    minNode = node
            visited[minNode] = True

            # update distance
            for edge in self.worldGraph.edges:
                if edge[0] == minNode and not visited[edge[1]] and not markedNodes[edge[1]]:
                    if distance[minNode] + 1 < distance[edge[1]]:
                        distance[edge[1]] = distance[minNode] + 1
                        prev[edge[1]] = minNode

        # find the path
        path = []
        node = end
        while node != start:
            path.append(node)
            node = prev[node]
        path.append(start)
        path.reverse()
        return path[1]

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
        (goal_Col, goal_Row) = self.checkPoints[chkPtsSoFar]
        print((goal_Col, goal_Row))
        (next_row, next_col) = self.getShortestPathUsingBFS((util.yToRow(curr_y), util.xToCol(curr_x)), (goal_Row, goal_Col), beliefOfOtherCars)
        goalPos = (util.colToX(next_col), util.rowToY(next_row)) # next tile
        moveForward = True

         
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
       
        time.sleep(0.1)

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
    
    