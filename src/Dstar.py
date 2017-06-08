# -*- coding: utf-8 -*-
"""
Created on Thu Mar 16 11:04:31 2017

@author: Sith, Andrew, Evan
"""

import sys
import math
import Queue
from graphics import Graphics

class Cell:

    # Access cell values:
    #   h:      path cost
    #   k:      smallest value of h seen so far
    #   b:      back pointer
    #   t:      tag ('c' - closed; 'o' - open; 'n' - new)
    def __init__(self, h, k, b, t, loc):
        self.h = h
        self.k = k
        self.b = b
        self.t = t
        self.loc = loc


class D_Star:
    
    # Init function with robot world's decomposition
    # Start (x,y) and Goal (x,y) 
    def __init__(self, size, start, goal):
        world_x = size[0]
        world_y = size[1]
        
        self.start = start
        self.goal = goal
        self.size = size
        self.world = [[Cell(None, None, None, 'n', (i, j)) for i in range(world_x)] for j in range(world_y)]
        self.pqueue = Queue.PriorityQueue() # Open list
        self.costs = [[[[None for _ in range(world_x)] for _ in range(world_y)] for _ in range(world_x)] for _ in range(world_y)]
        self.obstacles = []

        for j1 in range(world_y):
            for i1 in range(world_x):
                for j2 in range(world_y):
                    for i2 in range(world_x):
                        self.costs[j1][i1][j2][i2] = math.hypot(i2 - i1, j2 - j1)

    # Helper functions below
    # n denotes some tuple (i, j)
    # curr denotes some Cell object

    def get(self, n):
        return self.world[n[1]][n[0]]

    def put(self, n, cell):
        self.world[n[1]][n[0]] = cell

    def get_open(self):
        if not self.pqueue.empty():
            item = self.pqueue.get()
            curr = self.get(item[1])
            curr.t = 'c'

            return (item[0], curr)

        else:
            return (None, None)

    def put_open(self, cell):
        cell.t = 'o'
        
        self.pqueue.put((cell.h, cell.loc))

    def get_kmin(self):
        try:
            return sorted(self.pqueue.queue)[0][0]
        except:
            print "Unexpected error:", sys.exc_info()[0]
            return None

    def get_cost(self, curr1, curr2):
        x1, y1 = curr1.loc
        x2, y2 = curr2.loc

        return self.costs[y2][x2][y1][x1]

    def get_path(self, curr):
        path = [curr.loc]

        print "getting path from {}".format(curr.loc)
        print "open list: {}".format(self.pqueue.queue)

        while True:
            curr = curr.b

            if curr == None:
                return None
            else:
                print curr.loc
                if curr.loc in path:
                    raise Exception('Recursive backlinks')
                path.append(curr.loc)

                if curr.loc == self.goal:
                    return path

    def get_neighbors(self, curr):
        neighbors = []
        loc = curr.loc
        x = loc[0]
        y = loc[1]

        min_y = max(0, y - 1)
        max_y = min(self.size[1] - 1, y + 1)
        min_x = max(0, x - 1)
        max_x = min(self.size[0] - 1, x + 1)                
        
        for j in xrange(min_y, max_y + 1):
            for i in xrange(min_x, max_x + 1):
                if (i, j) == (x, y):
                    continue
                neighbors.append(self.get((i, j)))

        return neighbors

    ##################################################

    def init_path(self):
        goal = Cell(0, 0, 'g', 'o', self.goal)
        goal.b = goal
        self.put(self.goal, goal)
        self.put_open(goal)

        while True:
            k_min = self.process_state()
            print "Got k_min: {}".format(k_min)
            start = self.get(self.start)

            if k_min == None or start.t == 'c':
                return self.get_path(start)

    def change_map(self, actual_map, curr):
        for sensed in self.get_neighbors(curr):
            i, j = sensed.loc
            if actual_map[j][i]:
                self.insert(sensed, sensed.k)
                for neighbor in self.get_neighbors(sensed):
                    if self.costs[j][i][neighbor.loc[1]][neighbor.loc[0]] != float("inf"):
                        if not (i, j) in self.obstacles:
                            print "found obstacle: {}".format((i, j))
                            self.obstacles.append((i, j))
                        self.modify_costs(neighbor, sensed, float("inf"))

    def navigate_map(self, curr):
        while True:
            k_min = self.process_state()

            print "Got k_min: {}".format(k_min)

            if (k_min == None or curr.h <= k_min) and not curr.t == 'o':
                return self.get_path(curr)

    def modify_costs(self, curr1, curr2, new_c):
        x1, y1 = curr1.loc
        x2, y2 = curr2.loc
        self.costs[y2][x2][y1][x1] = new_c
        self.costs[y1][x1][y2][x2] = new_c

        if curr1.t == 'c':
            self.insert(curr1, curr1.h)

        return self.get_kmin()

    def insert(self, curr, h_new):
        print "inserting {} {} {}".format(curr.loc, curr.t, h_new)
        if curr.t == 'n':
            curr.h = h_new
            curr.k = h_new
            curr.t = 'o'
            self.put_open(curr)
        elif curr.t == 'o':
            curr.h = h_new
            curr.k = min(curr.k, h_new)
            indexes = [i for i,x in enumerate(self.pqueue.queue) if x[1] == curr.loc]
            for index in indexes:
                self.pqueue.queue[index] = (curr.k, curr.loc)
        elif curr.t == 'c':
            curr.h = h_new
            curr.k = min(curr.h, h_new)
            curr.t = 'o'
            self.put_open(curr)
    

    def process_state(self):
        k_old, curr = self.get_open()
        
        if k_old == None:
            print "popped {} from the open queue".format(k_old)
            return None

        print "popped ({}, {}) from the open queue with h: {}".format(k_old, curr.loc, curr.h)

        if k_old < curr.h:
            print "Raise {}".format(curr.loc)
            for neighbor in self.get_neighbors(curr):
                if neighbor.t != 'n' and neighbor.h <= k_old \
                and curr.h > neighbor.h + self.get_cost(neighbor, curr):
                    curr.b = neighbor
                    curr.h = neighbor.h + self.get_cost(neighbor, curr)

        if k_old == curr.h:
            for neighbor in self.get_neighbors(curr):
                if (neighbor.t == 'n') \
                or (neighbor.b == curr and neighbor.h != curr.h + self.get_cost(curr, neighbor)) \
                or (neighbor.b != curr and neighbor.h > curr.h + self.get_cost(curr, neighbor)):
                    neighbor.b = curr
                    self.insert(neighbor, curr.h + self.get_cost(curr, neighbor))

        else:
            for neighbor in self.get_neighbors(curr):
                if neighbor.t == 'n' \
                or (neighbor.b == curr and neighbor.h != curr.h + self.get_cost(curr, neighbor)):
                    neighbor.b = curr
                    self.insert(neighbor, curr.h + self.get_cost(curr, neighbor))

                elif neighbor.b != curr and neighbor.h > curr.h + self.get_cost(curr, neighbor):
                    self.insert(curr, curr.h)

                elif neighbor.b != curr and curr.h > neighbor.h + self.get_cost(curr, neighbor) \
                and neighbor.t == 'c' and neighbor.h > k_old:
                    self.insert(neighbor, neighbor.h)

        return self.get_kmin()

    def run(self, actual_map, size):
        final_path = [self.start]
        graphics = Graphics(actual_map, self.start, self.goal, size=size)
        graphics.display(final_path, self.obstacles)

        path = self.init_path()
        if path == None:
            return None

        curr = self.get(self.start)

        while curr.loc != self.goal:
            self.change_map(actual_map, curr)
            path = self.navigate_map(curr)

            if path == None:
                return None

            curr = self.get(path[1])
            final_path.append(path[1])

            graphics.display(final_path, self.obstacles)

        graphics.display(final_path, self.obstacles, done=True)
        return final_path
