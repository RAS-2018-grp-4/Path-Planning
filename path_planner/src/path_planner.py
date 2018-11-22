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
        self.theta = 0              # orientation

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
        self.map_minx = 0
        self.map_miny = 0

        # odometry and grid position
        self.x_start_odom = 0 # [m]
        self.y_start_odom = 0 # [m]
        self.x_start_grid = 0 # [1]
        self.y_start_grid = 0 # [1]

        # target position
        self.x_target_odom = 0 # [m]
        self.y_target_odom = 0 # [m]
        self.x_target_grid = 0 # [1]
        self.y_target_grid = 0 # [1]

    def Main(self):
        while not rospy.is_shutdown():
            rate.sleep()

    def mapCallback(self, msg):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map = msg.data
        self.map_resolution = msg.info.resolution
        self.map_minx = msg.info.origin.position.x
        self.map_miny = msg.info.origin.position.y
        #print(self.map_minx, self.map_miny)

    def odomCallback(self, msg):
        pass
        '''
        # current position as starting position
        self.x_start_odom = msg.pose.pose.position.x + 0.2
        self.y_start_odom = msg.pose.pose.position.y + 0.2
        self.x_start_grid = (int)(self.x_start_odom/self.map_resolution)
        self.y_start_grid = (int)(self.y_start_odom/self.map_resolution)
        #print(self.x_start_grid, self.y_start_grid)
        '''

    def filterCallback(self, msg):
        #pass
        #'''
        # current position as starting position
        
        self.x_start_odom = msg.pose.pose.position.x# + 0.2
        self.y_start_odom = msg.pose.pose.position.y# + 0.2

        self.x_start_grid = (int)((self.x_start_odom - self.map_minx)/self.map_resolution)
        self.y_start_grid = (int)((self.y_start_odom - self.map_miny)/self.map_resolution)
        #print(self.x_start_grid, self.y_start_grid)
        #'''

    def new_start(self, point):
        # manual starting position
        self.x_start_odom = point[0]
        self.y_start_odom = point[1]
        self.x_start_grid = (int)((self.x_start_odom - self.map_minx)/self.map_resolution)
        self.y_start_grid = (int)((self.y_start_odom - self.map_miny)/self.map_resolution)

    def new_target(self, point):
        self.x_target_odom = point[0]
        self.y_target_odom = point[1]
        self.x_target_grid = (int)((self.x_target_odom - self.map_minx)/self.map_resolution) 
        self.y_target_grid = (int)((self.y_target_odom - self.map_miny)/self.map_resolution)

    def obstacle_collision(self, x, y):
        status = False
        status = (self.map[x + y*self.map_width] == 100) or (self.map[x + y*self.map_width] == -2)
        return status

    def update_target(self, msg):
        print("new target aquired")
        point = (msg.pose.position.x, msg.pose.position.y)
        self.new_target(point)
        self.execute_planner()
 
    def execute_planner(self):
        path, visited = self.A_star()

        print("starting cell:", self.x_start_grid, self.y_start_grid)
        print("path (no smoothing):", path)

        path_smooth = self.smooth_path(path)
        print("path (with smoothing)", path)

        # construct the path
        rviz_path = Path()
        rviz_path.header.frame_id = 'map' 

        for c in path_smooth:
            #path_x = c[0]*pp.map_resolution
            #path_y = c[1]*pp.map_resolution
            path_x = c[0]
            path_y = c[1]

            pose = PoseStamped()
            #pose.header.frame_id = 'map' 
            pose.pose.position.x = path_x
            pose.pose.position.y = path_y
            rviz_path.poses.append(pose)

        '''
        for c in path:
            path_x = c[0]*pp.map_resolution
            path_y = c[1]*pp.map_resolution


            pose = PoseStamped()
            #pose.header.frame_id = 'map' 
            pose.pose.position.x = path_x
            pose.pose.position.y = path_y
            rviz_path.poses.append(pose)
        '''
        path_pub.publish(rviz_path)

        print("goal cell:", self.x_target_grid, self.y_target_grid)

    def euclidian_dist(self, x, y, xt, yt):
        h = np.sqrt((x - xt)**2 + (y - yt)**2) 
        return h

    def get_closest_free_space(self, x0, y0):
        print("getting new grid cell")
        free_cell = False
        r = 1
        while not free_cell:
            for motion in [(-r, 0), (0, r), (r, 0), (0, -r), (-r, r), (r, r), (r, -r), (-r, -r)]:
                if self.obstacle_collision(x0 + motion[0], y0 + motion[0]):
                    continue
                else:
                    print("grid cell fix DONE")
                    return (x0 + motion[0], y0 + motion[0])
            r = r + 1

    def A_star(self):
        print("hello")
        # environment bounds [m]
        xlb = 0
        xub = self.map_width
        ylb = 0
        yub = self.map_height

        # inital position [m]
        x0 = self.x_start_grid
        y0 = self.y_start_grid
        
        # if starting position not in free space, adjust the starting node
        if self.obstacle_collision(x0, y0):
            x0, y0 = self.get_closest_free_space(x0, y0)
        
        # target coordinates [m]
        xt = self.x_target_grid
        yt = self.y_target_grid

        # if goal position not in free space, adjust the goal node
        #if self.obstacle_collision(xt, yt):
        #    xt, yt = self.get_closest_free_space(xt, yt)

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
                    path.append((current.x, current.y))         # append the path
                    current = current.parent
                return path[::-1], visited[::-1]                # retrun the path (in reversed order)

            # generate new children (apply actions to the current best node)
            motion_long = [(-2, 0), (0, 2), (2, 0), (0, -2), (-2, 2), (2, 2), (2, -2), (-2, -2)]
            motion_short = [(-1, 0), (0, 1), (1, 0), (0, -1), (-1, 1), (1, 1), (1, -1), (-1, -1)]

            motions = []
            if self.euclidian_dist(current_node.x, current_node.x, end_node.x, end_node.y) <= 4:
                motions = motion_short
            else:
                motions = motion_long
            motions = motion_short


            children = []
            for motion in motions:
                # get the new node state
                node_state = (current_node.x + motion[0], current_node.y + motion[1]) # simulate the motion

                # check that the position is within the bounds
                if node_state[0] > xub or node_state[0] < xlb or node_state[1] > yub or node_state[1] < ylb:
                    continue

                # check that the position is not on obstacle
                #print(node_state[0], node_state[1], end_node.x, end_node.y)
                if self.euclidian_dist(node_state[0], node_state[1], end_node.x, end_node.y) <= 2: 
                    print("close to goal, let planner go into yellow area")
                else:
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
                child.g = current_node.g + move_cost*0.3                                # path length penalty
                child.h = self.euclidian_dist(child.x, child.y, end_node.x, end_node.y) # euclidian distance
                child.f = child.g + child.h                                             # total cost

                # check if child is already in the open list and compare cost
                for open_node in alive_list:
                    if child == open_node and child.g >= open_node.g:
                        skip_child = True
                        break

                if skip_child:
                    continue # skip this child

                # if the child is neither in the closed list nor open list, add it to the open list
                alive_list.append(child)

        # if no path found
        path = []
        current = current_node
        print("NO PATH FOUND!")
        return [], []               # retrun the path (in reversed order)

    def smooth_path(self, path):
        #####################
        # to do: 1) remove spurious "reverse lines" 2) fix evenly spaced path dots
        #####################
        smooth_path = []

        path_x = []
        path_y = []

        ray_length = 3

        last_ok = path[0] # last reached grid cell on the original path
        start = path[0] # start grid to search 


        counter = 0
        idx = 1
        while True:
            counter = counter + 1
            if idx >= len(path) - 1 or counter > len(path)*4:
                # add the last bit of the ray trace
                path_x = np.linspace(start[0]*self.map_resolution + self.map_minx, last_ok[0]*self.map_resolution + self.map_minx, num=ray_length, endpoint=True)
                path_y = np.linspace(start[1]*self.map_resolution + self.map_miny, last_ok[1]*self.map_resolution + self.map_miny, num=ray_length, endpoint=True)
                for i in range(ray_length):
                    smooth_path.append((path_x[i], path_y[i]))

                # exit the loop
                break

            # perform raytrace
            free_path, collision = self.raytrace(start, path[idx])
                 
            if not collision:      
                # last reached grid cell (without collision)
                last_ok = path[idx]
                ray_length = 3*len(free_path)
                idx = idx + 1

            # if collision draw a straight line from the current start to the last_ok grid cell
            else:   
                if len(free_path) <= 2:                 
                    path_x = np.linspace(start[0]*self.map_resolution + self.map_minx, path[idx][0]*self.map_resolution + self.map_minx, num=ray_length, endpoint=True)
                    path_y = np.linspace(start[1]*self.map_resolution + self.map_miny, path[idx][1]*self.map_resolution + self.map_miny, num=ray_length, endpoint=True)
                    # update ray start to the last grid cell
                    start = path[idx]
                    idx = idx + len(free_path)

                elif len(free_path) > 2:     
                    path_x = np.linspace(start[0]*self.map_resolution + self.map_minx, last_ok[0]*self.map_resolution + self.map_minx, num=ray_length, endpoint=True)
                    path_y = np.linspace(start[1]*self.map_resolution + self.map_miny, last_ok[1]*self.map_resolution + self.map_miny, num=ray_length, endpoint=True)
                    # update ray start to the last_ok grid cell
                    start = last_ok

                # append the straight line
                for i in range(ray_length):
                    smooth_path.append((path_x[i], path_y[i]))

        
        '''
        for c in path[1:]:
            free_path, collision = self.raytrace(start, c)

            if not collision: 
                # last reached grid cell (without collision)
                last_ok = c 

            # if collision draw a straight line from the current start to the last_ok grid cell
            if collision:     
                ray_length = 3 + 3*int(np.sqrt((start[0] - start[0])**2 + (last_ok[0] - last_ok[1])**2))     
                path_x = np.linspace(start[0]*self.map_resolution, last_ok[0]*self.map_resolution, num=ray_length, endpoint=True)
                path_y = np.linspace(start[1]*self.map_resolution, last_ok[1]*self.map_resolution, num=ray_length, endpoint=True)
                for i in range(ray_length):
                    smooth_path.append((path_x[i], path_y[i]))

                # update ray start to the last_ok grid cell
                start = last_ok
  
        # add the last bit of the ray trace
        path_x = np.linspace(start[0]*self.map_resolution, last_ok[0]*self.map_resolution, num=ray_length, endpoint=True)
        path_y = np.linspace(start[1]*self.map_resolution, last_ok[1]*self.map_resolution, num=ray_length, endpoint=True)
        for i in range(ray_length):
            smooth_path.append((path_x[i], path_y[i]))
        '''

        # smooth the path with a averaging window
        n_window = 10
        smooth_path_final = smooth_path

        for i in range(n_window/2+1, len(smooth_path) - n_window - 1):
            sumx = 0
            sumy = 0
            for j in range(-n_window/2,n_window/2):
                sumx = sumx + smooth_path[i+j][0]
                sumy = sumy + smooth_path[i+j][1]
            smooth_path_final[i] = (sumx/n_window, sumy/n_window)
     
        return smooth_path_final

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
            if self.obstacle_collision(int(x), int(y)):# and self.euclidian_dist(int(x), int(y), self.x_target_grid, self.y_target_grid) >= 2:
                return traversed, True
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    if self.obstacle_collision(int(x + x_inc), int(y)):# and self.euclidian_dist(int(x), int(y), self.x_target_grid, self.y_target_grid) >= 2:
                        return traversed, True
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed, False



if __name__ == '__main__':
    print("path planner started")

    pp = PathPlanner()
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, pp.update_target)
    rospy.Subscriber("/maze_map_node/map", OccupancyGrid, pp.mapCallback)
    #rospy.Subscriber("/robot_odom", Odometry, pp.odomCallback)
    rospy.Subscriber("/robot_filter", Odometry, pp.filterCallback)
    path_pub = rospy.Publisher('/aPath', Path, queue_size=10)

    time.sleep(2)
    #pp.new_start([0.2, 0.2])
    #pp.new_target([2.2, 0.8])
    #pp.execute_planner()
    pp.Main()
    

    '''
    pp.new_start([0.2, 0.2])
    pp.new_target([2.2, 0.8])
    pp.execute_planner()
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



