# -*- coding:utf-8 -*-
import math
import numpy as np


'''Stores attributes of map

Variables:
    min_x, min_y, max_x, max_y: border coordinates of the map in original units
    og_width, og_height: width/height of the map in orginal units
    width, height: width/height of the map in unit robot length
    robot_len: length of robot
    vis[(x,y)]: visited times block (x,y)
    blocked[(x,y)]: is 1 if (x,y) is blocked, 0 otherwise
    graph[(x,y,i)]: is 1 if there is an edge from (x,y) in i direction, that is
                    an edge from (x,y) to (x+dir[i][0], y+dir[i][1])
    block_num: # blocks to be visited in the whole graph
    recorder: temporary stores the number of blocks already visited
'''
class Map():
    def __init__(self):
        self.min_x = -1
        self.min_y = -1
        self.max_x = -1
        self.max_y = -1
        self.og_width = -1
        self.og_height = -1
        self.width = -1
        self.height = -1
        self.robot_len = 0.3

        self.vis = {}
        self.blocked = {}
        self.graph = {}

        self.block_num = 0
        # TODO: recorder now stores the number of block visited 
        # because depth() isn't working at this moment, 
        # should change back to depth() once it's fixed
        self.recorder = 1
    
    # shows a graph of borders(blocked) and spanning tree(vis)
    def show(self):
        import matplotlib.pyplot as plt
        blockedx = []
        blockedy = []
        for i in range(0, self.width+1):
            for j in range(0, self.height+1):
                if (self.blocked[(i,j)]):
                    # print (i,j)
                    blockedx.append(i * self.robot_len + self.min_x)
                    blockedy.append(j * self.robot_len + self.min_y)
        plt.plot(blockedx, blockedy, '.')

        visx = []
        visy = []
        for i in range(0, self.width+1):
            for j in range(0, self.height+1):
                if (self.vis[(i,j)]):
                    visx.append(i * self.robot_len + self.min_x)
                    visy.append(j * self.robot_len + self.min_y)
        plt.plot(visx, visy, '*')

        plt.show()


    # initialize most attributes in Map, including map.height, width, vis, blocked
    # @params [filename] is the .smap file we are reading in
    def initialize_map(self, filename):
        from MapReader import MapReader
        
        import sys
        import os

        map_reader = MapReader(filename)
        map_reader.run()

        map_x = map_reader.map_x
        map_y = map_reader.map_y

        self.min_x = math.floor(min(map_reader.map_x))
        self.max_x = math.ceil(max(map_reader.map_x))
        self.og_width = self.max_x - self.min_x
        self.min_y = math.floor(min(map_reader.map_y))
        self.max_y = math.ceil(max(map_reader.map_y))
        self.og_height = self.max_y - self.min_y

        self.width = math.floor(self.og_width / self.robot_len)
        self.height = math.floor(self.og_height / self.robot_len)

        # initialize all the blocks in range
        for i in range(1,self.width+1):
            for j in range(1,self.height+1):
                self.vis[(i,j)] = 0
                self.blocked[(i,j)] = 0

        # mark all blocks with obstacles as blocked
        for i in range(len(map_reader.map_x)):
            self.blocked[self.to_point_coor(map_reader.map_x[i], map_reader.map_y[i])] = 1

        # initializethe borders to avoid array out of bound 
        for i in range(-10,self.width+10):
            for j in range(-10, self.height+10):
                if(self.is_valid_point(i,j)):
                    self.vis[(i,j)] = 0
                    self.blocked[(i,j)] = 0
                else :
                    self.vis[(i,j)] = 0
                    self.blocked[(i,j)] = 1
        
        # and count valid block numbers
        for i in range(0,self.width+2, 2):
            for j in range(0, self.height+2, 2):
                if(self.is_valid_block(i,j)):
                    self.block_num = self.block_num + 1

        # Only for small graph testing purposes:
        # Find an empty point where we can start generating spanning tree
        # start_x = -1
        # start_y = -1
        # initialized = False
        # for i in range(1,self.width+1):
        #     if initialized:
        #         break
        #     for j in range(1,self.height+1):
        #         (block_x, block_y) = point_to_block(i,j)
        #         #if not (self.vis[(i,j)] or self.blocked[(i,j)] or initialized):
        #         if self.is_valid_block(block_x, block_y) and not initialized:
        #             start_x = i
        #             start_y = j
        #             print(start_x,start_y)
        #             initialized = True
        #             break


    '''converts coordinates in map unit to our robot_length unit

    First moves the whole graph to the origin by subtracting them with min_x, min_y
    then normalizes these point into robot_length unit so that they can be 
    represented with integer index
    '''
    def to_point_coor(self, mapx, mapy):
        return ( 
            math.floor((mapx-self.min_x)/self.robot_len), 
            math.floor((mapy-self.min_y)/self.robot_len))


    # @return true when position (x,y) is within bound of the map and is not blocked
    def is_valid_point(self, x, y):
        if (x,y) in self.blocked:
            return (not self.blocked[(x,y)]) \
                and x>0 and x<self.width \
                and y>0 and y<self.height
        else :
            return False

    # @return true if all four points in this block are valid
    def is_valid_block(self, x, y):
        return x % 2 == 0 and y % 2 == 0 and \
            self.is_valid_point(x-1,y-1) and \
            self.is_valid_point(x-1,y) and \
            self.is_valid_point(x,y-1) and \
            self.is_valid_point(x,y)

'''A block class representing big blocks, containing 4 points

Each block is a node in this quadtree map, a unit of later generated 
spanning tree. Each node contains its own coordinate, the parent of this node, 
and four children of this block, in a direction of E S W N

A block always has an even number index. Block (i,j) contians point 
(i-1, j-1) (i, j-1) (i-1, j) (i, j), as in self.is_valid_block(x,y).
For example, block(2,2) contains point (1,1) (1,2) (2,1) (2,2)
'''
class Block:
    def __init__(self, ix, iy):
        self.x = ix
        self.y = iy
        self.parent = None
        self.children = np.array([None, None, None, None])


'''A point class representing the unit position of a robot

Four points constitute a block. A point's length is the same as the robot length.
'''
class Point:
    def __init__(self,*args):
        if len(args)==0:
            self.x = -1
            self.y = -1
        elif len(args) == 2:
            self.x = args[0]
            self.y = args[1]
        self.last = None
        self.next = None

# gets the block that point (ptx, pty) belongs to
def point_to_block(ptx, pty):
    return ((ptx+1) //2 *2, (pty+1)//2 *2)