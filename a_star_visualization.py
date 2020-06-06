#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  8 18:46:45 2020

@author: lahmwang
"""

import pygame
import math
import os


width = 600
height = 600

num_cells = 40
# size of gap in between cells
gap_size = width // 600
# size of each square cell
cell_size = (width // num_cells) - gap_size

# color initialization
grey = (100, 100, 100)
white = (255, 255, 255)
black = (0, 0, 0)
orange = (240, 140, 0)
blue = (10, 120, 200)
green = (0, 240, 15)
red = (240, 0, 0)

# frame per second
speed = 60

window = pygame.display.set_mode((width, height))
pygame.display.set_caption("A* Shortest-Pathfinding Algorithm Visualizer")

# a class for all the cells in the grid
class Cell():

    def __init__(self, parent = None, relation = None, position = None):
        self.parent = parent
        self.relation = relation
        self.position = position
        self.f = 0
        self.g = 0
        self.h = 0
        
    # see if the position overlaps
    def __eq__(self, other):
        return self.position == other.position

def Initialize():
    grid = [[0 for i in range(num_cells)] for j in range(num_cells)]
    # whether the a* is running or it is initializing
    running = False
    # initializing start and end node
    start = None
    end = None
    # control the loop of initialization
    initialize_done = False
    draw_grid(grid, None, None, None, None, running, False)
    while not initialize_done:
        for event in pygame.event.get():
            # if click the cross on the corner or hit esc button, exit
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                pygame.quit()
                os._exit(0)
            mousex, mousey = pygame.mouse.get_pos()
            # left mouse: set start point
            if pygame.mouse.get_pressed()[0]:
                start = get_coord(mousex, mousey)
            # right mouse: set end point
            elif pygame.mouse.get_pressed()[2]:
                end = get_coord(mousex, mousey)
            # middle mouse: set obstacle
            elif pygame.mouse.get_pressed()[1]:
                x, y = get_coord(mousex, mousey)
                try:
                    # label the obstacle as 1
                    grid[y][x] = 1
                except:
                    continue
            # hit whitespace or enter to start visualization
            if event.type == pygame.KEYDOWN and (event.key == pygame.K_SPACE or event.key == pygame.K_RETURN):
                initialize_done = True

        draw_grid(grid, start, end, None, None, running, False)
        pygame.display.update()

    A_star(grid, start, end)


def A_star(grid, start, end):
    running = True
    finish = False
    # create start and end node as Cell instances
    start_node = Cell(None, None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Cell(None, None, end)
    end_node.g = end_node.h = end_node.f = 0
    # the points that we are to consider during this process
    open_list = []
    # the points that are already visited, no longer processing
    closed_list = []
    # list storing the nodes along the path
    path = []
    open_list.append(start_node)

    while not finish:

        current_node = open_list[0]
        current_index = 0
        for index in range(len(open_list)):
            now = open_list[index]
            # the node with smallest f value within the open list -> current node
            if now.f < current_node.f:
                current_node = now
                current_index = index

        # move the current node from open list to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # stop the loop until find the ending point
        if current_node == end_node:
            current = current_node
            print("The shortest path length is:", format(current.f, ".4f"))
            # backtrack to parents until reaching the starting point to get the path
            while current is not None:
                path.append(current.position)
                current = current.parent
            finish = True

        adjacent = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            try:
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            except:
                print("Invalid Position Coordinate Appears")
                pygame.quit()
                os._exit(0)

            # make sure the coordinate is within the grid
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) - 1) or node_position[1] < 0:
                continue

            # make sure the coordinate doesn't represent an obstacle
            if grid[node_position[1]][node_position[0]] != 0:
                continue

            # create new adjacent node and add to the adjacent list
            # if this cell is on the diagonal position of current node (of distance sqrt(2))
            if abs(new_position[0]) + abs(new_position[1]) == 2:
                new_node = Cell(current_node, "diagonal", node_position)
            # else, this cell is on the left/right/up/down position of current node (of distance 1)
            else:
                new_node = Cell(current_node, "alongside", node_position)
            adjacent.append(new_node)

        # loop through the adjacent list, update every node's f,g,h value, and decide whether to append this node to open list (for next-step processing)
        for adj in adjacent:
            append = True
            
            # if this cell is on closed list, don't append it
            if adj in closed_list:
                append = False

            # based on the "relation" attribute, update the g value
            if adj.relation == "diagonal":
                adj.g = current_node.g + math.sqrt(2)
            else:
                adj.g = current_node.g + 1
                
            # use euclidean distance to calculate h value
            try:
                adj.h = math.sqrt(((adj.position[0] - end_node.position[0]) ** 2) + ((adj.position[1] - end_node.position[1]) ** 2))
            except:
                print("Invalid Distance Calculation Appears")
                pygame.quit()
                os._exit(0)
                
            # sum up to get the f value
            adj.f = adj.g + adj.h

            # if this node is already in open list and it has larger g value, don't append it
            if adj in open_list:
                append = False

            if append:
                open_list.append(adj)
        
        # update the new grids
        draw_grid(grid, start, end, open_list, closed_list, running, finish, path)
        
        # see if the open list becomes empty before the ending point is found
        try:
            game_loop(open_list[0], finish)
        except IndexError:
            print("No Valid Path")
            pygame.quit()
            os._exit(0)

# control the execution before/during/after the Pygame program
# no big relationship with the a* algorithm itself
def game_loop(cell, finish):
    clock = pygame.time.Clock()
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            os._exit(0)
    # control whether to play the game again
    if finish:
        # control the current loop of execution of the game
        while finish:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    os._exit(0)
                # press whitespace or enter to play again
                if (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE) or (event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN):
                    finish = False
            pygame.display.update()
        # restart the program
        Initialize()
    pygame.display.update()
    clock.tick(speed)

# draw the current status of the grids on the window
def draw_grid(grid, start, end, open_list, closed_list, running, finish, path = None):
    window.fill(white)
    for y_index, y_cell in enumerate(grid):
        for x_index, x_cell in enumerate(y_cell):
            left, top = get_top_left_pixel_coord(x_index, y_index)
            pygame.draw.rect(window, grey, (left, top, cell_size, cell_size))
            
            # paint the starting point with red
            if start is not None and x_index == start[0] and y_index == start[1]:
                pygame.draw.rect(window, red, (left, top, cell_size, cell_size))
            # paint the ending point with red
            if end is not None and x_index == end[0] and y_index == end[1]:
                pygame.draw.rect(window, red, (left, top, cell_size, cell_size))
            # paint the obstacles with black
            if x_cell == 1:
                pygame.draw.rect(window, black, (left, top, cell_size, cell_size))

            if running:
                # paint the open list elements with orange
                for node in open_list:
                    if node.position[0] == x_index and node.position[1] == y_index:
                        pygame.draw.rect(window, orange, (left, top, cell_size, cell_size))
                # paint the closed list elements with blue
                for node in closed_list:
                    if node.position[0] == x_index and node.position[1] == y_index:
                        pygame.draw.rect(window, blue, (left, top, cell_size, cell_size))
                if finish:
                    # paint the path with green
                    for n in path:
                        if n[0] == x_index and n[1] == y_index:
                            pygame.draw.rect(window, green, (left, top, cell_size, cell_size))
                            
# convert the pixel expression to the grid cell's index (in the array)
def get_coord(x, y):
    for node_x in range(num_cells):
        for node_y in range(num_cells):
            left, top = get_top_left_pixel_coord(node_x, node_y)
            # store the rectangle area
            Rectangle = pygame.Rect(left, top, cell_size, cell_size)
            # see if the x,y coordinate is in the rectangle
            if Rectangle.collidepoint(x, y):
                return (node_x, node_y)
    return (None, None)

# convert the grid cell's index to the pixel coordinate (of the cell's top left corner)
def get_top_left_pixel_coord(x, y):
    left = x * (cell_size + gap_size)
    top = y * (cell_size + gap_size)
    return (left, top)


def main():
    pygame.init()
    Initialize()


if __name__ == "__main__":
    main()