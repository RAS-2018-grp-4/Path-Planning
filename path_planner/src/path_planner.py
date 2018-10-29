#!/usr/bin/env python

import numpy as np

#import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import time
import matplotlib.pyplot as plt
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from math import cos, sin, atan2, fabs, sqrt
import numpy as np

PoseStamped

rospy.init_node('path_planner_node', anonymous=True)
rate = rospy.Rate(1)

class Node():
    def __init__(self, parent = None, state = None, movement = (0,0)):
        self.parent = parent        # previous state
        self.x = state[0]           # x position
        self.y = state[1]           # y position
        self.movement = (movement[0], movement[1]) # movement from previous to this state
        
        self.g = 0 # path length cost
        self.h = 0 # euclidian distance to target cost
        self.f = 0 # combined cost

    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)


class PathPlanner():
    def __init__(self):
        # map settings
        self.map = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0

        # odometry and grid position
        self.x_start_odom = 0 # [m]
        self.y_start_odom = 0 # [m]
        self.x_start_grid = 0 # [1]
        self.y_start_grid = 0 # [1]

        # target position
        self.x_target_odom = 0
        self.y_target_odom = 0
        self.x_target_grid = 0
        self.y_target_grid = 0

    def mapCallback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map = msg.data
        self.map_resolution = msg.info.resolution

    def add_to_map(self, x, y, val):
        self.map[x + y*self.map_width] = val

    '''
    def odomCallback(self, msg):
        self.x_start_odom = msg.pose.pose.position.x
        self.y_start_odom = msg.pose.pose.position.y
        self.x_start_grid = (int)(self.x_start_odom/self.map_resolution)
        self.y_start_grid = (int)(self.y_start_odom/self.map_resolution)
    '''

    def update_target(self, msg):
        point = (msg.pose.position.x, msg.pose.position.y)
        self.new_target(point)

        # find new path
        pp.new_start([0.2, 0.2])
        #pp.new_target([2.2, 0.8])
        path, visited = pp.A_star()
        print(path)

        path = pp.smooth_path(path)
        print(path)

        # construct the path
        rviz_path = Path()
        rviz_path.header.frame_id = 'map' 

        for c in path:
            #path_x = c[0]*pp.map_resolution
            #path_y = c[1]*pp.map_resolution
            path_x = c[0]
            path_y = c[1]
            #print(path_x, path_y)

            pose = PoseStamped()
            #pose.header.frame_id = 'map' 
            pose.pose.position.x = path_x
            pose.pose.position.y = path_y
            rviz_path.poses.append(pose)

        path_pub.publish(rviz_path)
        print(path)
        print("goal:", pp.x_target_grid, pp.y_target_grid)

    def new_start(self, point):
        self.x_start_odom = point[0]
        self.y_start_odom = point[1]
        self.x_start_grid = (int)(self.x_start_odom/self.map_resolution)
        self.y_start_grid = (int)(self.y_start_odom/self.map_resolution)

    def new_target(self, point):
        self.x_target_odom = point[0]
        self.y_target_odom = point[1]
        self.x_target_grid = (int)(self.x_target_odom/self.map_resolution) 
        self.y_target_grid = (int)(self.y_target_odom/self.map_resolution)

    def obstacle_collision(self, x, y):
        status = False
        status = (self.map[x + y*self.map_width] == 100) or (self.map[x + y*self.map_width] == -2)
        return status

    def euclidian_dist(self, x, y, xt, yt):
        h = np.sqrt((x - xt)**2 + (y - yt)**2) 
        return h

    def A_star(self):
        # environment bounds [m]
        xlb = 0
        xub = self.map_width
        ylb = 0
        yub = self.map_height

        # inital position [m]
        x0 = self.x_start_grid
        y0 = self.y_start_grid
        
        # target coordinates [m]
        xt = self.x_target_grid
        yt = self.y_target_grid

        # initialize start and end node
        start_node = Node(None, (x0, y0))
        end_node = Node(None, (xt, yt))

        # initialize open and closed list
        alive_list = []
        dead_list = []

        # append the start node to the open list
        alive_list.append(start_node)

        visited = []
        iter = 0
        # run algorithm until the end node (target state) is found
        while len(alive_list) > 0:
            iter = iter + 1
            # get the current node (first item in list)
            current_node = alive_list[0]
            current_idx = 0

            # pick the node with the lowest total cost
            for idx, item in enumerate(alive_list):
                if item.f < current_node.f:
                    current_node = item
                    current_idx = idx
                
            #print("new node at", current_node.x, current_node.y)
            visited.append((current_node.x, current_node.y))

            # remove the current node from the open list, and add it to the closed list
            alive_list.pop(current_idx)
            dead_list.append(current_node)


            # check if the current node is on the goal
            if current_node == end_node or iter > 5000:        
                path = []
                current = current_node
                while current is not None:
                    path.append((current.x, current.y))    # append the path
                    current = current.parent
                return path[::-1], visited[::-1]               # retrun the path (in reversed order)

            # generate new children (apply actions to the current best node)
            children = []

            for motion in [(-1, 0), (0, 1), (1, 0), (0, -1), (-1, 1), (1, 1), (1, -1), (-1, -1)]:
                #print(motion)

                # get the new node state
                node_state = (current_node.x + motion[0], current_node.y + motion[1]) # simulate the motion

                # check that the position is within the bounds
                if node_state[0] > xub or node_state[0] < xlb or node_state[1] > yub or node_state[1] < ylb:
                    continue

                # check that the position is not on obstacle
                if self.obstacle_collision(node_state[0], node_state[1]):
                    continue

                # if the criteria above are fulfilled, create and append the node
                new_node = Node(current_node, node_state, motion)
                children.append(new_node)

            # loop through the children and compare with already alive and dead nodes
            for child in children:
                skip_child = False
                # check if child is on the closed list
                for closed_child in dead_list:
                    if child == closed_child:
                        skip_child = True
                        break 

                if skip_child:
                    continue # skip this child

                # generate the f, g, h metrics
                move_cost = np.sqrt(child.movement[0]**2 + child.movement[1]**2)

                child.g = current_node.g + move_cost*0.2                                        # path length penalty
                child.h = self.euclidian_dist(child.x, child.y, end_node.x, end_node.y)  # euclidian distance
                child.f = child.g + child.h                               # total cost

                # check if child is already in the open list and compare cost
                for open_node in alive_list:
                    if child == open_node and child.g >= open_node.g:
                        skip_child = True
                        break

                if skip_child:
                    continue # skip this child

                # if the child is neither in the closed list nor open list, add it to the open list
                alive_list.append(child)

    def smooth_path(self, path):
        smooth_path = []

        start = path[0]
        idx = 0
        for c in path[1:]:
            free_path, collision = self.raytrace(start, c)
            if collision:
                path_x = np.linspace(start[0]*self.map_resolution, free_path[-1][0]*self.map_resolution, num=10, endpoint=True)
                path_y = np.linspace(start[1]*self.map_resolution, free_path[-1][1]*self.map_resolution, num=10, endpoint=True)
                for i in range(10):
                    smooth_path.append((path_x[i], path_y[i]))
                idx = idx + len(free_path)
                start = c
        

        return smooth_path

    def raytrace(self, start, end):
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):      
            if self.obstacle_collision(int(x), int(y)) and i >= 2:
                return traversed, True
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    if self.obstacle_collision(int(x + x_inc), int(y)) and i >= 2:
                        return traversed, True
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return None, False



if __name__ == '__main__':
    print("path planner started")

    pp = PathPlanner()
    rospy.Subscriber('move_base_simple/goal', PoseStamped, pp.update_target)
    rospy.Subscriber("/maze_map_node/map", OccupancyGrid, pp.mapCallback)
    #rospy.Subscriber("/robot_odom", Odometry, pp.odomCallback)
    path_pub = rospy.Publisher('/aPath', Path, queue_size=10)

    time.sleep(3)

    '''
    pp.new_start([0.2, 0.2])
    pp.new_target([2.2, 0.8])
    path, visited = pp.A_star()
    print(path)

    path = pp.smooth_path(path)
    print(path)

    # construct the path
    rviz_path = Path()
    rviz_path.header.frame_id = 'map' 

    for c in path:
        #path_x = c[0]*pp.map_resolution
        #path_y = c[1]*pp.map_resolution
        path_x = c[0]
        path_y = c[1]
        #print(path_x, path_y)

        pose = PoseStamped()
        #pose.header.frame_id = 'map' 
        pose.pose.position.x = path_x
        pose.pose.position.y = path_y
        rviz_path.poses.append(pose)
    path_pub.publish(rviz_path)


    print(path)
    print("goal:", pp.x_target_grid, pp.y_target_grid)
    '''

    '''
    path_x = [c[0] for c in path]
    path_y = [c[1] for c in path]
    plt.plot(path_x, path_y, label="course")

    for (x, y) in visited:
        plt.plot(x, y, marker='o')
    

    plt.axis("equal")
    plt.grid(True)
    plt.show()
    '''



