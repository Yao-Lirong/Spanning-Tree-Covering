# -*- coding:utf-8 -*-

'''
Coverage Path Planning Algorithm (implemented by Spiral Tree Planning)
Generates spanning tree path to allow robot to fully cover the map given.

Author: Yao Lirong
Reference: 
    http://planning.cs.uiuc.edu/node353.html
    https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/SpiralSpanningTreeCPP/spiral_spanning_tree_coverage_path_planner.py

This algorithm now only implements a priliminary STC(spanning tree converage)
algorithm described in this paper (https://ieeexplore.ieee.org/abstract/document/1013479). 
The author purposefully paints small blocks blocked instead of big blocks blocked
to make a later implementation of full STC possible.

Coordinate Index:
    All coordinates start with 1, 0 is a border and shouldn't contain anything.
    Points in original map are converted to abstracted block and point that can 
    be accessed with an integer index. One unit "point" has the same length as 
    the robot's, because it represents the location of robot. One unit "block"
    has twice the robot's length, becuase one "block" is a unit of our spanning
    tree and contains four "points".
    A block always has an even number index. Block (i,j) contians point 
    (i-1, j-1) (i, j-1) (i-1, j) (i, j), as in is_valid_block(x,y).

Robot Location:
    A legal robot position is always 
        1. at the end point of the quadtree edge 
           it's moving along, but not pass this end point.
        2. robot's right side is wall
    This invariant always holds after robot's position is initialized. All four 
    moving methods (move_direct, turn_left, turn_right, turn_around) also 
    preserve this invaraint

Direction:
    Sequence of direction array should not be changed without a second thought.
    Once it's changed, four move function, initialize_position(), 
    initialize_graph(), and draw_path() should all be changed accordingly. 

Importnat:
    1. depth() is not working
    2. st() now stops processing when it can't proceed, instead of repeating the 
       process until all blocks are visited. I adopt this solution because 
       sometimes not all blocks can be reached. 

TODO:
    We used to have 
    
'''
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import json as js
import math
import queue
import matplotlib.pyplot as plt



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
map = Map()
# direction in the sequence E S W N
direction = [[1,0],[0,-1],[-1,0],[0,1]]
# prints out debugging info of st(spanning tree) to console if set to True
DEBUGst = False
# prints out debugging info of draw_path to console if set to True
DEBUGdp = False

'''A block class representing big blocks, containing 4 points

Each block is a node in this quadtree map, a unit of later generated 
spanning tree. Each node contains its own coordinate, the parent of this node, 
and four children of this block, in a direction of E S W N

A block always has an even number index. Block (i,j) contians point 
(i-1, j-1) (i, j-1) (i-1, j) (i, j), as in is_valid_block(x,y).
For example, block(2,2) contains point (1,1) (1,2) (2,1) (2,2)
'''
class Block:
    def __init__(self, ix, iy):
        self.x = ix
        self.y = iy
        self.parent = None
        self.children = np.array([None, None, None, None])
# root of the spanning tree
root = Block(-1,-1)

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

# starting position of the robot
start = Point(-1, -1)
robot_dir = 0

# Map Reader copied from [MapWidget.py]
class Readmap():
    def __init__(self, filename):
        self.map_name = filename
        self.js = dict()
        self.map_x = []
        self.map_y = []
        self.verts = []
        self.circles = []
        self.points = []
        self.straights = []
        self.res = -1
    # run method gets called when we start the thread
    def run(self):
        fid = open(self.map_name, encoding= 'UTF-8')
        self.js = js.load(fid)
        fid.close()
        self.map_x = []
        self.map_y = []
        self.verts = []
        self.circles = []
        self.straights = []
        self.points = []
        self.p_names = []
        self.res = self.js['header']['resolution']
        # print(self.js.keys())
        for pos in self.js['normalPosList']:
            if 'x' in pos:
                self.map_x.append(float(pos['x']))
            else:
                self.map_x.append(0.0)
            if 'y' in pos:
                self.map_y.append(float(pos['y']))
            else:
                self.map_y.append(0.0)
        if 'advancedCurveList' in self.js:
            for line in self.js['advancedCurveList']:
                if line['className'] == 'BezierPath':
                    x0 = 0
                    y0 = 0
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    x3 = 0
                    y3 = 0
                    if 'x' in line['startPos']['pos']:
                        x0 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y0 = line['startPos']['pos']['y']
                    if 'x' in line['controlPos1']:
                        x1 = line['controlPos1']['x']
                    if 'y' in line['controlPos1']:
                        y1 = line['controlPos1']['y']
                    if 'x' in line['controlPos2']:
                        x2 = line['controlPos2']['x']
                    if 'y' in line['controlPos2']:
                        y2 = line['controlPos2']['y']
                    if 'x' in line['endPos']['pos']:
                        x3 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y3 = line['endPos']['pos']['y']
                    self.verts.append([(x0,y0),(x1,y1),(x2,y2),(x3,y3)])
                elif line['className'] == 'ArcPath':
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    x3 = 0
                    y3 = 0
                    if 'x' in line['startPos']['pos']:
                        x1 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y1 = line['startPos']['pos']['y']
                    if 'x' in line['controlPos1']:
                        x2 = line['controlPos1']['x']
                    if 'y' in line['controlPos1']:
                        y2 = line['controlPos1']['y']
                    if 'x' in line['endPos']['pos']:
                        x3 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y3 = line['endPos']['pos']['y']
                    A = x1*(y2-y3) - y1*(x2-x3)+x2*y3-x3*y2
                    B = (x1*x1 + y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1)
                    C = (x1*x1 + y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+(x3*x3+y3*y3)*(x1-x2)
                    D = (x1*x1 + y1*y1)*(x3*y2-x2*y3)+(x2*x2+y2*y2)*(x1*y3-x3*y1)+(x3*x3+y3*y3)*(x2*y1-x1*y2)
                    if abs(A) > 1e-12:
                        x = -B/2/A
                        y = -C/2/A
                        r = math.sqrt((B*B+C*C-4*A*D)/(4*A*A))
                        theta1 = math.atan2(y1-y,x1-x)
                        theta3 = math.atan2(y3-y,x3-x)
                        v1 = np.array([x2-x1,y2-y1])
                        v2 = np.array([x3-x2,y3-y2])
                        flag = float(np.cross(v1,v2))
                        if flag >= 0:
                            self.circles.append([x, y, r, np.rad2deg(theta1), np.rad2deg(theta3)])
                        else:
                            self.circles.append([x, y, r, np.rad2deg(theta3), np.rad2deg(theta1)])
                    else:
                        self.straights.append([(x1,y1),(x3,y3)])
                elif line['className'] == 'StraightPath':
                    x1 = 0
                    y1 = 0
                    x2 = 0
                    y2 = 0
                    if 'x' in line['startPos']['pos']:
                        x1 = line['startPos']['pos']['x']
                    if 'y' in line['startPos']['pos']:
                        y1 = line['startPos']['pos']['y']
                    if 'x' in line['endPos']['pos']:
                        x2 = line['endPos']['pos']['x']
                    if 'y' in line['endPos']['pos']:
                        y2 = line['endPos']['pos']['y']
                    self.straights.append([(x1,y1),(x2,y2)])
        if 'advancedPointList' in self.js:
            for pt in self.js['advancedPointList']:
                x0 = 0
                y0 = 0 
                theta = 0
                if 'x' in pt['pos']:
                    x0 = pt['pos']['x']
                if 'y' in pt['pos']:
                    y0 = pt['pos']['y']
                if 'dir' in pt:
                    theta = pt['dir']
                if  'ignoreDir' in pt:
                    if pt['ignoreDir'] == True:
                        theta = None
                self.points.append([x0,y0,theta])
                self.p_names.append([pt['instanceName']])

# @return true when position (x,y) is within bound of the map and is not blocked
def is_valid_point(x, y):
    if (x,y) in map.blocked:
        return (not map.blocked[(x,y)]) \
            and x>0 and x<map.width \
            and y>0 and y<map.height
    else :
        return False

# @return true if all four points in this block are valid
def is_valid_block(x, y):
    return x % 2 == 0 and y % 2 == 0 and \
        is_valid_point(x-1,y-1) and \
        is_valid_point(x-1,y) and \
        is_valid_point(x,y-1) and \
        is_valid_point(x,y)

# @return the number of nodes from this node
# TODO: not working 
def depth(node):
    def depth_aux(n, dpth):
        for i in range(4):
            if (n.children[i]!=None):
                return depth_aux(n.children[i], dpth+1)
            else :
                return dpth
    return depth_aux(node, 0)

# generate a spanning tree from robot's position
# IMPORTANT: it now stops generating when it backtraces to root and finds no 
#            next possible node. Earlier it stops when all blocks are visited
def st():
    now = root
    map.vis[(now.x,now.y)] = 1
    while (True):
        if (DEBUGst):
            print("now at (%d, %d) %d" %(now.x, now.y, map.vis[(now.x,now.y)]))
        if (DEBUGst):
            print("current depth is %d" %(depth(root)))
            print("what is this then %d" %(map.recorder))

        # stop generating if all nodes are found
        # if (map.recorder == map.block_num): 
        #     print("all nodes found")
        #     break

        # looking for blocks that have not been visited
        found = False
        for i in range(4):
            nextx = now.x + direction[i][0] * 2
            nexty = now.y + direction[i][1] * 2
            if (DEBUGst):
                print("next is (%d, %d)" %(nextx, nexty))
            if (is_valid_block(nextx,nexty) and map.vis[(nextx,nexty)]<1):
                next = Block(nextx, nexty)
                map.vis[(nextx,nexty)] = map.vis[(nextx,nexty)] + 1
                next.parent = now
                now.children[i] = next
                now = next
                found = True
                map.recorder = map.recorder + 1
                break
        
        # if we cannot find a node that hasn't been visited from current node,
        # we should start tracing back to the first node satisfying condition
        if (not found): 
            if DEBUGst:
                print ("tracing back")
            new_found = False

            while (True):
                now = now.parent
                # Edited: Rather than meaning no solution, it means path  
                #         generating has finished 
                if (now == None):
                    return
                    # raise Exception("no solution")
                for i in range(4):
                    nextx = now.x + direction[i][0] * 2
                    nexty = now.y + direction[i][1] * 2
                    if (DEBUGst):
                        print("next is (%d, %d) %d " %(nextx, nexty,map.vis[(nextx,nexty)]))
                    if (is_valid_block(nextx,nexty) and map.vis[(nextx,nexty)]<1):
                        next = Block(nextx, nexty)
                        map.vis[(nextx,nexty)] = map.vis[(nextx,nexty)] + 1
                        next.parent = now
                        now.children[i] = next
                        now = next
                        new_found = True
                        map.recorder = map.recorder + 1
                        break
                if new_found:
                    break

# print out the quadtree starting from [node] to console
# This function isn't always working as it requires many layers of recursion
def show_sp(node):
    print("now at (" + str(node.x) + ", " + str(node.y) +") ")
    has_children = False
    for i in range(4):
        if(node.children[i] != None):
            has_children = True
            print(
                "(" + str(node.x) + ", " + str(node.y) +") ->" \
                + "(" + str(node.children[i].x) + ", " \
                + str(node.children[i].y) +")")
            show_sp(node.children[i])
    if not has_children:
        print("(" + str(node.x) + ", " + str(node.y) +") ->" + "None")


def move_direct(now, dr):
    if (DEBUGdp):
        print("trying to move direct ...")
    next =  Point();
    if (dr == 1 or dr == 3) :
        next.x = now.x;
        next.y = now.y + (-2 if dr == 1 else 2)
    elif ( dr == 0 or dr == 2):
        next.y = now.y;
        next.x = now.x + (2 if dr == 0 else -2)
    else:
        raise Exception ("dr is not in the range [0,3]")

    next.last = now;
    now.next = next;

    if (DEBUGdp):
        print("(%d, %d)" %(next.x, next.y))
    return next;

def turn_right(now, dr):
    if (DEBUGdp):
        print("trying to turn right...")
    toturn = Point()
    turned = Point()

    if dr == 0: 
        toturn.x = now.x + 1
        toturn.y = now.y
        turned.x = toturn.x
        turned.y = toturn.y - 2
    elif dr == 1:
        toturn.x = now.x
        toturn.y = now.y - 1
        turned.x = toturn.x - 2
        turned.y = toturn.y
    elif dr == 2:
        toturn.x = now.x - 1
        toturn.y = now.y
        turned.x = toturn.x
        turned.y = toturn.y + 2
    elif dr == 3:
        toturn.x = now.x
        toturn.y = now.y + 1
        turned.x = toturn.x + 2
        turned.y = toturn.y
    else:
        raise Exception ("dr is not in the range [0,3]")

    toturn.last = now
    now.next = toturn
    turned.last = toturn
    toturn.next = turned
    if (DEBUGdp):
        print("(%d, %d)" %(toturn.x, toturn.y))
        print("(%d, %d)" %(turned.x, turned.y))
    return turned

def turn_left(now, dr):
    if (DEBUGdp):
        print("trying to turn left...")
    next = Point()
    if (dr == 1 or dr == 3):
        next.x = now.x + (1 if dr==1 else -1)
        next.y = now.y
    elif (dr == 0 or dr == 2):
        next.x = now.x
        next.y = now.y + (1 if dr==0 else -1)
    else:
        raise Exception ("dr is not in the range [0,3]")
    next.last = now
    now.next = next
    if (DEBUGdp):
        print("(%d, %d)" %(next.x, next.y))
    return next

def turn_around(now, dr):
    if (DEBUGdp):
        print("trying to turn around...")
    turn1 = Point()
    turn2 = Point()
    turned = Point()
    if dr == 0:
        turn1.x = now.x + 1
        turn1.y = now.y
        turn2.x = turn1.x
        turn2.y = turn1.y - 1
        turned.x = turn2.x - 2
        turned.y = turn2.y
    elif dr == 1:
        turn1.x = now.x
        turn1.y = now.y - 1
        turn2.x = turn1.x - 1
        turn2.y = turn1.y
        turned.x = turn2.x
        turned.y = turn2.y + 2
    elif dr == 2:
        turn1.x = now.x - 1
        turn1.y = now.y
        turn2.x = turn1.x
        turn2.y = turn1.y + 1
        turned.x = turn2.x + 2
        turned.y = turn2.y
    elif dr == 3:
        turn1.x = now.x
        turn1.y = now.y + 1
        turn2.x = turn1.x + 1
        turn2.y = turn1.y
        turned.x = turn2.x
        turned.y = turn2.y - 2
    else:
        raise Exception ("dr is not in the range [0,3]")

    turn1.last = now
    now.next = turn1
    turn2.last = turn1
    turn1.next = turn2
    turned.last = turn2
    turn2.next = turned
    if (DEBUGdp):
        print("(%d, %d)" %(turn1.x, turn1.y))
        print("(%d, %d)" %(turn2.x, turn2.y))
        print("(%d, %d)" %(turned.x, turned.y))

    return turned

# @return initialized robot position as a point object 
#         and initialized robot direction
# initialize the robot's position according to the first edge from root node
# the initialized position also follows robot position rule specifed in the
# head comment(not surpassing edge end node, wall always on robot's right side)
def initialize_position():

    if map.graph[(root.x, root.y, 0)]:
        return (Point(root.x+1, root.y), 0)
    elif map.graph[(root.x, root.y, 1)]:
        return (Point(root.x, root.y-1), 1)
    elif map.graph[(root.x, root.y, 2)]:
        return (Point(root.x-2, root.y), 2)
    elif map.graph[(root.x, root.y, 3)]:
        return (Point(root.x-1, root.y+1), 3)
    else :
        raise Exception ("No edge from toot, Graph not initialized")

# initialize map.graph, robot position, and robot direction
# used a queue to avoid exceeding maximu recursion level
def initialize_graph():
    for i in range(0, map.width+2):
        for j in range(0, map.height+2):
            for k in range(4):
                map.graph[(i, j, k)] = 0
    q = queue.Queue()
    def initialize_graph_aux(now):
        for i in range(4):
            next = now.children[i]
            if (next!=None) :
                map.graph[(now.x, now.y, i)] = 1
                map.graph[(next.x, next.y, (i+2)%4)] = 1
                q.put(next)
    initialize_graph_aux(root)
    while not q.empty() :
        initialize_graph_aux(q.get())

    global start, robot_dir
    (start, robot_dir) = initialize_position()

'''Draws the robot's path around generated spanning tree

draw_path() first initializes a graph of wall(spanning tree), along which the
robot runs, and the robot's initial position.
It then starts to draw path. Recall the wall is always on the robot's right side.
    1. When we encounter a wall ahead of us, we turn left.
    2. When the wall extends along with direction unchanged, 
       we follow this wall and go direct.
    3. When the wall turns right, the robot turns right too.
    4. When none of the above condition is satisfied, 
       we reached the end of an edge and should turn around to the other side.
It repeats the above 4 actions until it completes a round trip along the wall, 
that's when it reaches back to the initial position.
'''
def draw_path():
    initialize_graph()
    now = start
    global robot_dir
    

    if DEBUGdp:
        print("now at (%d, %d)" %(now.x, now.y))

    do = True
    while (now.x != start.x or now.y != start.y) or do:
        do = False
        if DEBUGdp:
            print("now at (%d, %d) facing %d" %(now.x, now.y, robot_dir))
        (block_x, block_y) = point_to_block(now.x, now.y)
        # block_x = get_block_coor(now.x)
        # block_y = get_block_coor(now.y)

        #TODO: comment 
        if map.graph[(block_x, block_y, (robot_dir+3)%4)]: #先判定左孩子，其实是 (dir-1)%4
            now = turn_left(now, robot_dir)
            robot_dir = (robot_dir+3) % 4
        elif map.graph[(block_x, block_y, (robot_dir))]:
            now = move_direct(now, robot_dir)
        elif map.graph[(block_x, block_y, (robot_dir+1)%4)]:
            now = turn_right(now, robot_dir)
            robot_dir = (robot_dir+1) % 4
        elif map.graph[(block_x, block_y, (robot_dir+2)%4)]:
            now = turn_around(now, robot_dir)
            robot_dir = (robot_dir+2) % 4
        else :
            raise Exception ("no solution")

'''converts coordinates in map unit to our robot_length unit

First moves the whole graph to the origin by subtracting them with min_x, min_y
then normalizes these point into robot_length unit so that they can be 
represented with integer index
'''
def to_point_coor(mapx, mapy):
    return ( 
        math.floor((mapx-map.min_x)/map.robot_len), 
        math.floor((mapy-map.min_y)/map.robot_len))

# gets the block that point (ptx, pty) belongs to
def point_to_block(ptx, pty):
    return ((ptx+1) //2 *2, (pty+1)//2 *2)

# initialize most attributes in Map, including map.height, width, vis, blocked
# @params (rx, ry) is the robot's location (starting location)
def initialize_map(filename, rx, ry):
    import sys
    import os

    read_map = Readmap(filename)
    read_map.run()

    map_x = read_map.map_x
    map_y = read_map.map_y

    map.min_x = math.floor(min(read_map.map_x))
    map.max_x = math.ceil(max(read_map.map_x))
    map.og_width = map.max_x - map.min_x
    map.min_y = math.floor(min(read_map.map_y))
    map.max_y = math.ceil(max(read_map.map_y))
    map.og_height = map.max_y - map.min_y

    map.width = math.floor(map.og_width / map.robot_len)
    map.height = math.floor(map.og_height / map.robot_len)

    # initialize all the blocks in range
    for i in range(1,map.width+1):
        for j in range(1,map.height+1):
            map.vis[(i,j)] = 0
            map.blocked[(i,j)] = 0

    # mark all blocks with obstacles as blocked
    for i in range(len(read_map.map_x)):
        map.blocked[to_point_coor(read_map.map_x[i], read_map.map_y[i])] = 1

    # initializethe borders to avoid array out of bound 
    for i in range(-10,map.width+10):
        for j in range(-10, map.height+10):
            if(is_valid_point(i,j)):
                map.vis[(i,j)] = 0
                map.blocked[(i,j)] = 0
            else :
                map.vis[(i,j)] = 0
                map.blocked[(i,j)] = 1
    
    # and count valid block numbers
    for i in range(0,map.width+2, 2):
        for j in range(0, map.height+2, 2):
            if(is_valid_block(i,j)):
                map.block_num = map.block_num + 1

    # Only for small graph testing purposes:
    # Find an empty point where we can start generating spanning tree
    # start_x = -1
    # start_y = -1
    # initialized = False
    # for i in range(1,map.width+1):
    #     if initialized:
    #         break
    #     for j in range(1,map.height+1):
    #         (block_x, block_y) = point_to_block(i,j)
    #         #if not (map.vis[(i,j)] or map.blocked[(i,j)] or initialized):
    #         if is_valid_block(block_x, block_y) and not initialized:
    #             start_x = i
    #             start_y = j
    #             print(start_x,start_y)
    #             initialized = True
    #             break

    (start_x, start_y) = to_point_coor(rx, ry)
    global root
    root = Block(
        point_to_block(start_x,start_y)[0],
        point_to_block(start_x,start_y)[1])

# convert robot's path, stored in linked list, to a python list
def output_result():
    result = []
    now = start
    do = True
    while (now.x != start.x or now.y != start.y) or do:
        do = False
        # result.append((now.x * map.robot_len, now.y * map.robot_len))
        result.append((now.x, now.y))
        now = now.next
    return result

# shows a graph of borders(blocked), spanning tree(vis), and robot's path(path)
def show_graph():
    blockedx = []
    blockedy = []
    for i in range(0, map.width+1):
        for j in range(0, map.height+1):
            if (map.blocked[(i,j)]):
                # print (i,j)
                blockedx.append(i * map.robot_len + map.min_x)
                blockedy.append(j * map.robot_len + map.min_y)
    plt.plot(blockedx, blockedy, '.')

    # visx = []
    # visy = []
    # for i in range(0, map.width+1):
    #     for j in range(0, map.height+1):
    #         if (map.vis[(i,j)]):
    #             visx.append(i * map.robot_len + map.min_x)
    #             visy.append(j * map.robot_len + map.min_y)
    # plt.plot(visx, visy, '*')
    
    pathx = []
    pathy = []
    result = output_result()
    for i in result:
        pathx.append(i[0] * map.robot_len + map.min_x)
        pathy.append(i[1] * map.robot_len + map.min_y)
    plt.plot(pathx, pathy, '.-')

    plt.show()

def main():

    initialize_map("D://Programming//Seer_Robot//My Implementation//3_v2.smap", 20, 10)
    st()
    # show_sp(root)
    draw_path()
    show_graph()




if __name__ == "__main__":
    main()


'''
Reference
1. [Global Variable in Python](https://blog.csdn.net/qq_28888837/article/details/88060376)
    引用全局变量，不需要golbal声明，修改全局变量，需要使用global声明
2. [print 函数用法总结](https://www.runoob.com/w3cnote/python3-print-func-b.html)
3. [check if key is in dictionary](https://thispointer.com/python-how-to-check-if-a-key-exists-in-dictionary/)
4. [Python中的三则运算符](https://www.cnblogs.com/mywood/p/7416893.html)
5. Python 没有 `switch` 语句
6. [Python模拟 do while loop](https://www.jquery-az.com/what-is-python-do-while-loop/)
7. [Python Comment Style Guide](https://google.github.io/styleguide/pyguide.html)
'''