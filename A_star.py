import numpy as np
import matplotlib.pyplot as plt
from operator import add
import sympy as sy
from sympy.polys.polyfuncs import interpolate
import heapq
import pdb
from itertools import count
import timeit
from random import shuffle, randrange
from random import randint


class A_Star:
    def __init__(self,obs,start,end):
        self.start_node=start
        self.end_node=end
        self.save_simulation = True
        self.obstacle=obs

    # Total Heurestic Cost 
    def heurestic_cost(self, start,end):
        return np.hypot(start[:,0]-end[:,0],start[:,1]-end[:,1])

    #Possible Motion Available
    def motion_primitive(self):
        return [[0,1],[1,1],[1,0],[-1,0],[0,-1],[-1,-1],[-1,1],[1,-1]]

    # Cost of taking a Motion
    def motion_cost(self):
        return np.array([1.0,1.4,1.0,1.0,1.0,1.4,1.4,1.4])

    # Calculates f=g+h
    def total_cost(self,parent_cost,motion_cost,heurestic_cost):
        return parent_cost + motion_cost + heurestic_cost

    # Returns the path
    def get_path(self,closed_list):
        curren_node_parent=closed_list[tuple(self.end_node)]['parent_node']
        path=[]
        path.append(curren_node_parent)
        while not np.all(np.array(curren_node_parent)==np.array(self.start_node)):
            curren_node_parent=closed_list[tuple(curren_node_parent)]['parent_node']
            path.append(curren_node_parent)
        
        return path

    def A_Star(self):
        
        #huerestic cost
        h=self.heurestic_cost( np.array([self.start_node]),np.array([self.end_node]))

        # Distance Cost
        g=0 

        # Total Cost
        f=self.total_cost(0,g,h)

        #imgct - imgcounter for animation
        imgct=0

        #Priority Queue will keep the list sorted so that we pop the heurestically best node
        open_list = []
        counter=0
        heapq.heappush(open_list,[h,counter,dict({"child_node":self.start_node,"parent_node":self.start_node,'heurestic_cost':h,'cost_to_go':0})]);
        
        #Maps all open list nodes.To ensure we don't have duplicates in the priority queue we check this
        open_list_map={}
        open_list_map[tuple(self.start_node)]=1

        # Closed list is selected as dictionary to get O(1) searching
        closed_list= dict()

        # Children List is hash map. For each child key we store a parent. Each child has a unique parent.
        # Graph created should not be cyclic
        children =dict()

        # Capture vehicle motion
        motion_primitives= np.array(self.motion_primitive())
        motion_cost=self.motion_cost()  

        # Loop through priorityqueue
        while open_list:

            node=heapq.heappop(open_list)[2]

            # Add the node to the closed list as we visited it
            closed_list[tuple(node['child_node'])]=dict({'parent_node':node['parent_node'],'cost':node['heurestic_cost']+node['cost_to_go']})
            open_list_map.pop(tuple(node['child_node']))
            
            #  Exit condition
            if(np.all(node['child_node']==np.array(self.end_node))):
                print("Goal Reached here")
                return self.get_path(closed_list)          
            
            #child node becomes new parent node
            new_child_nodes=np.array(node['child_node'])+motion_primitives[:]

            # We calculate the new heuristic cost, cost to go and total cost
            heurestic_cost=self.heurestic_cost(np.array(new_child_nodes),np.array([self.end_node]))
            cost_to_go=node['cost_to_go']+motion_cost 
            total_cost=self.total_cost(np.array(node['cost_to_go']),motion_cost,heurestic_cost)

            """ f=g+h 
                f:Total cost 
                g:cost to go
                h:heuristic cost"""
            for child,f,h,g in zip(new_child_nodes,total_cost,heurestic_cost,cost_to_go):
                
                # if it is in obstacle skip
                if self.obstacle.get(tuple(child),-1) ==1:
                    continue
                
                #check if in closed list
                #if yes check current cost and based on that update the cost and parent
                if closed_list.get(tuple(child),-1)!=-1:
                    if closed_list[tuple(child)]['cost']>f:
                         closed_list[tuple(child)]=dict({'parent_node':node['child_node'],'cost':f})
                         print("rewired")
                else:
                    if open_list_map.get(tuple(child),-1) ==1:
                        continue
                    else:
                        counter=counter+1
                        open_list_map[tuple(child)]=1
                        heapq.heappush(open_list,[f,counter,dict({"child_node":child,"parent_node":node['child_node'],"cost_to_go":g,"heurestic_cost":h})]);
                    
            imgct+=1
            if self.save_simulation and imgct%50==0:
                cl=np.asarray([*closed_list])           
                plt.plot(cl[:,0],cl[:,1],'.b')
                op=np.asarray([*open_list_map])
                plt.plot(op[:,0],op[:,1],'.g')
              
                plt.pause(0.1)
                plt.savefig('astar_'+str(imgct))
                


def path(pts,startX,stopX):
    
    x=sy.Symbol('x')
    y= sy.interpolate(pts,x)
    x=np.linspace(startX,stopX,100)
    def f(x):
        return y
    
    return f(x)
  

def design():
    road={}
    for i in range(51):
        road[i,45]=1
        road[i,5]=1
     
    for i in range(-5,56):
        road[i,0]=1
        road[i,50]=1
     
  
    for i in range(5,45):
        road[50,i]=1
        road[0,i]=1
    
    for i in range(0,50):
        road[55,i]=1
        road[-5,i]=1
    return road
    
def creat_map(start_node,end_node):
    # Set the size of the maze.
    # These must be odd integers of 7 or above.
    xSize = 67
    ySize = 35

    # Set the maximum wall length.  This avoids overly long
    # walls on larger mazes.
    # The smaller the value, the greater possible solutions.
    maxWall = 13

    # Create a list to store the maze data.
    # 0 denotes a walkway, 1 is a wall, start with no walls.
    maze = (xSize*ySize)*[0]

    # Set every other cell to 2, these are the starting points.
    for i in range(0,xSize,2):
        for j in range(0,ySize,2):
            
            maze[(j*xSize)+i]=2

    # Create a wall around the maze.
    # Top and bottom.
    for i in range(0,xSize):
        maze[i]=1
        maze[(xSize*(ySize-1))+i] = 1
    # Left and right.
    for i in range(1,ySize-1):
        maze[xSize*i]=1
        maze[(xSize*i)+(xSize-1)] = 1

    # Create the maze.
    while 2 in maze:

        # Pick a random starting point.
        x = randint(1,(xSize-1)/2)*2
        y = randint(1,(ySize-1)/2)*2
        c = maze[(y*xSize)+x]

       

        # If the point hasn't already been used pick a random direction
        # and move that way until we hit a wall.
        if c == 2:
            mwc = 0
            r = randint(0,3)
            if r == 0:
                while c != 1:
                    maze[(y*xSize)+x] = 1
                    y -= 1
                    c = maze[(y*xSize)+x]
                    mwc += 1
                    if mwc == maxWall: break
            if r == 1:
                while c != 1:
                    maze[(y*xSize)+x] = 1
                    x += 1
                    c = maze[(y*xSize)+x]
                    mwc += 1
                    if mwc == maxWall: break
            if r == 2:
                while c != 1:
                    maze[(y*xSize)+x] = 1
                    y += 1
                    c = maze[(y*xSize)+x]
                    mwc += 1
                    if mwc == maxWall: break
            if r == 3:
                while c!= 1:
                    maze[(y*xSize)+x] = 1
                    x -= 1
                    c = maze[(y*xSize)+x]
                    mwc += 1
                    if mwc == maxWall: break

    maze_map={}
    arr=np.ones((xSize,ySize))
    for i in range(0,ySize): 
        for j in range(0,xSize):
            k = maze[(i*xSize)+j]
            if k == 1:
                if (i==start_node[0]  and j==start_node[1]) or (j==end_node[1]and i==end_node[0]):
                    continue
                else:
                    maze_map[(i,j)]=1
                    arr[j][i]=0
            
    return maze_map,arr
        
def get_astar_path(sx,sy,ex,ey):
    
    obs_=design()
    obs,arr=creat_map([sx,sy],[ex,ey])
    plt.plot(ex,ey,'xm')
    plt.plot(sx,sy,'xm')

    path_planner=A_Star(obs,[sx,sy],[ex, ey])

    plt.pcolormesh(arr)
    plt.axes().set_aspect('equal') #set the x and y axes to the same scale
    plt.xticks([]) # remove the tick marks by setting to an empty list
    plt.yticks([]) # remove the tick marks by setting to an empty list
    plt.axes().invert_yaxis() #invert the y-axis so the first row of data is at the top

    

    start_time=timeit.default_timer()
    path=path_planner.A_Star() 
    if path!=None:
        nppath=np.asarray(path)
        end_time=timeit.default_timer()

        print("timing",end_time-start_time)

        plt.plot(nppath[:,0],nppath[:,1],'r-')
        plt.show()   
    else:
        print("path not found")
        plt.show()

    return path

if __name__ == '__main__':
    get_astar_path(sx=10,sy=2,ex=25,ey=60)  