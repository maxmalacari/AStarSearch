# A* search Algorithm - Max Malacari 20/02/2017
# Edited 16/05/2020
# Demo mode - create random obstacle fields and start/end points
# Includes random obstacles in the way
# Can't move diagonally "between" obstacles

import pygame as pg
import random as rand
from os import sys
import time

# Some options to set!
wWidth = 1600 # dimensions of the drawing window (nominal 700*700)
wHeight = 1000
fullScreen = True # do we want to run full screen?
cols = 80 # number of cells in each dimension (nominal 50*50)
rows = 50
wallFraction = 0.3 # fraction of cells that contain an obstacle (0.3 nominal)
showBoundaries = True # show bounding boxes for cells
showProcess = True # show the search process (slower)
pauseTime = 2 # seconds to pause after solving
seed = 0 # randomization seed
    
# Some colours
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
yellow = (255,255,0)
white = (255,255,255)
brown = (168,121,60)
purple = (150,26,217)
black = (0,0,0)
grey = (64, 65, 66)

w = wWidth / cols
h = wHeight / rows

pg.init()
pg.display.set_caption('A* Search Algorithm - Max Malacari')
if fullScreen:
    screen = pg.display.set_mode((wWidth,wHeight),pg.FULLSCREEN)
else:
    screen = pg.display.set_mode((wWidth,wHeight))
rand.seed(seed)

# Class to hold the properties of a cell
class Cell():
    def __init__(self, i, j, start_i, start_j, end_i, end_j):
        self.i = i
        self.j = j
        self.f = 0
        self.g = 0
        self.h = 0
        self.neighbours = []
        self.previous = 0
        self.isWall = False

        if rand.random() < wallFraction:
            self.isWall = True
        if i == start_i and j == start_j:
            self.isWall = False
        if i == end_i and j == end_j:
            self.isWall = False

    def show(self, colour):
        pg.draw.rect(screen, colour, (self.i*w, self.j*h, w, h), 0)

    def showCellBoundary(self):
        pg.draw.rect(screen, black, (self.i*w, self.j*h, w, h), 1)

    def addNeighbours(self, grid):
        if self.isWall == False: # only add neighbours to non-wall cells
            i = self.i
            j = self.j
            # top/bottom/left/right
            if i < cols-1 and grid[i+1][j].isWall == False:
                self.neighbours.append(grid[i+1][j])
            if j < rows-1 and grid[i][j+1].isWall == False:
                self.neighbours.append(grid[i][j+1])
            if i > 0 and grid[i-1][j].isWall == False:
                self.neighbours.append(grid[i-1][j])
            if j > 0 and grid[i][j-1].isWall == False:
                self.neighbours.append(grid[i][j-1])

def main():

    for i in range(100):
        print("Puzzle:", i+1)
        screen.fill(white)

        start_i = rand.randint(0,cols-1)
        start_j = rand.randint(0,rows-1)
        end_i = start_i
        end_j = start_j
        while (start_i == end_i) and (start_j == end_j):
            end_i = rand.randint(0,cols-1)
            end_j = rand.randint(0,rows-1)

        grid = []

        setup(grid, cols, rows, start_i, start_j, end_i, end_j)
        start = grid[start_i][start_j]
        end = grid[end_i][end_j]

        openSet = []
        closedSet = []
        wallSet = []
        for i in range(0,cols):
            for j in range(0,rows):
                if grid[i][j].isWall:
                    wallSet.append(grid[i][j])

        openSet.append(start)
        finished = False

        while finished==False:

            path = []
            # Show wall cells
            for i in range(0,len(wallSet)):
                wallSet[i].show(black)
            # Show cells in the open set
            for i in range(0,len(openSet)):
                openSet[i].show(green)
            # Show cells in the closed set
            for i in range(0,len(closedSet)):
                closedSet[i].show(red)

            # Algorithm
            if len(openSet) > 0:
                lowestCostIndex = 0
                current = openSet[lowestCostIndex]

                for i in range(0,len(openSet)):
                    if openSet[i].f < openSet[lowestCostIndex].f:
                        lowestCostIndex = i
                        current = openSet[lowestCostIndex]
                if current == end:
                    finished = True
                    path = calculatePath(current) # get optimal path
                    print("Solution found!")

                else:
                    closedSet.append(current)
                    openSet.remove(current)

                    neighbours = current.neighbours
                    for i in range(0,len(neighbours)):
                        neighbour = neighbours[i]
                        neighbour.h = heuristic(neighbour, end)
                        if neighbour in closedSet:
                            continue
                        temp_g = current.g + heuristic(neighbour, current) # movement cost to get to neighbour
                        if neighbour in openSet: # is neighbour already in the open set?
                            if temp_g < neighbour.g: # did we get there more efficiently?
                                neighbour.g = temp_g
                                neighbour.f = neighbour.h + neighbour.g
                                neighbour.previous = current
                        else:
                            neighbour.g = temp_g
                            neighbour.f = neighbour.h + neighbour.g
                            neighbour.previous = current
                            openSet.append(neighbour)

                        path = calculatePath(current) # get the current path to this cell

            else:
                finished = True # to quit the loop
                print("No solution!")

            if showProcess or finished:
                # Show the current path
                ptList = []
                for i in range(0,len(path)):
                    #path[i].show(blue)
                    ptList.append((path[i].i*w + w/2, path[i].j*h + h/2))
                if len(path) > 1:
                    showLine(ptList, blue)

                # Show bounding boxes
                if showBoundaries == True:
                    for i in range(0,cols):
                        for j in range(0,rows):
                            grid[i][j].showCellBoundary()

                # show end points
                start.show(brown)
                end.show(purple)

                # Update the display with new draw objects
                pg.display.update()
                pg.event.pump() # clears the event queue so things don't eventually freeze

                event = pg.event.poll()
                if event.type == pg.KEYUP:
                    pg.quit()
                    sys.exit()
                
                if finished:
                    time.sleep(pauseTime)




# Set up the grid of cell objects
def setup(grid, cols, rows, start_i, start_j, end_i, end_j):
    for i in range(0,cols):
        grid.append([])
        for j in range(0,rows):
            grid[i].append(Cell(i,j, start_i, start_j, end_i, end_j))

    # add neighbours once grid is initialized
    for i in range(0,cols):
        grid.append([])
        for j in range(0,rows):
            grid[i][j].addNeighbours(grid)

def heuristic(cell1, cell2):
    return abs(cell1.i - cell2.i) + abs(cell1.j - cell2.j)

def calculatePath(current): # get current optimal path up to current cell
    path = []
    temp = current
    path.append(temp)
    while temp.previous:
        path.append(temp.previous)
        temp = temp.previous
    return path

def showLine(ptList, colour):
    pg.draw.lines(screen, colour, False, ptList, 7)

main()
