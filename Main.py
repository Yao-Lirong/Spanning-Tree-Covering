

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
    2. spanning_tree() now stops processing when it can't proceed, instead of repeating the 
       process until all blocks are visited. I adopt this solution because 
       sometimes not all blocks can be reached. 
    
'''


from PathPlanner import *

# shows a graph of borders(blocked), spanning tree(vis), and robot's path(path)
def show_graph(map, path):
    import matplotlib.pyplot as plt
    blockedx = []
    blockedy = []
    for i in range(0, map.width+1):
        for j in range(0, map.height+1):
            if (map.blocked[(i,j)]):
                # print (i,j)
                blockedx.append(i * map.robot_len + map.min_x)
                blockedy.append(j * map.robot_len + map.min_y)
    plt.plot(blockedx, blockedy, '.')

    visx = []
    visy = []
    for i in range(0, map.width+1):
        for j in range(0, map.height+1):
            if (map.vis[(i,j)]):
                visx.append(i * map.robot_len + map.min_x)
                visy.append(j * map.robot_len + map.min_y)
    plt.plot(visx, visy, '*')
    
    pathx = []
    pathy = []
    for i in path:
        pathx.append(i[0] * map.robot_len + map.min_x)
        pathy.append(i[1] * map.robot_len + map.min_y)
    plt.plot(pathx, pathy, '.-')

    plt.show()

def main():

    # initialize a map object
    map_path = "D://Programming//Seer_Robot//My_Implementation//3_v2.smap"
    my_map = Map()
    my_map.initialize_map(map_path)
    
    # initialize a path planner by giving it map object and the coordinate of 
    # the robot in map's original unit. Then start generating spanning tree,
    # generating path, and drawing out resulted path graph
    planner = PathPlanner(my_map, 20, 10)
    planner.spanning_tree()
    # planner.show_sp(root)
    planner.draw_path()
    result = planner.get_path()
    show_graph(my_map, result)




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
8. [Importing Modules in Python](https://www.datacamp.com/community/tutorials/modules-in-python)
9. [Creating and Importing Modules in Python](https://stackabuse.com/creating-and-importing-modules-in-python/)
'''