
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

import Maze


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
        return [[0,1],[1,0],[-1,0],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]]

    # Cost of taking a Motion
    def motion_cost(self):
        return np.array([1.0,1.0,1.0,1.0,1.4,1.4,1.4,1.4])

    # Calculates f=g+h
    def total_cost(self,parent_cost,motion_cost,heurestic_cost):
        return parent_cost + motion_cost + heurestic_cost

    # Returns the path
    def get_path(self,closed_list):
        current_node_parent=closed_list[tuple(self.end_node)]['parent_node']
        path=[]
        path.append(self.end_node)
        path.append(current_node_parent)
        while not np.all(np.array(current_node_parent)==np.array(self.start_node)):
            current_node_parent=closed_list[tuple(current_node_parent)]['parent_node']
            path.append(current_node_parent)
        
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
        open_list_map[tuple(self.start_node)]=[h,counter,dict({"child_node":self.start_node,"parent_node":self.start_node,'heurestic_cost':h,'cost_to_go':0,'cost_to_go':0})]

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
            closed_list[tuple(node['child_node'])]=dict({'parent_node':node['parent_node'],'cost':node['heurestic_cost']+node['cost_to_go'],"cost_to_go":node['cost_to_go']})
      
            open_list_map.pop(tuple(node['child_node']),-1)
            
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
                    if closed_list[tuple(child)]['cost_to_go']>g:
                        closed_list[tuple(child)]=dict({'parent_node':node['child_node'],'cost':f,'cost_to_go':g})
                else:
                    if open_list_map.get(tuple(child),-1) !=-1:
                        if open_list_map[tuple(child)][2]['cost_to_go']>g:
                            # This removal is the slowest operation. Find a way to remove at O(1)

                            #Exception Handling Needs a closer look
                            try:
                                open_list.remove(open_list_map[tuple(child)])
                            except:
                                pass

                            open_list_map.pop(tuple(node['child_node']),-1)
                            open_list_map[tuple(child)]=[f,counter,dict({"child_node":child,"parent_node":node['child_node'],"cost_to_go":g,"heurestic_cost":h})]
                            counter=counter+1
                            heapq.heappush(open_list,[f,counter,dict({"child_node":child,"parent_node":node['child_node'],"cost_to_go":g,"heurestic_cost":h})]);
                        else:
                            continue
                    else:
                        counter=counter+1
                        open_list_map[tuple(child)]=[f,counter,dict({"child_node":child,"parent_node":node['child_node'],"cost_to_go":g,"heurestic_cost":h})]
                        heapq.heappush(open_list,[f,counter,dict({"child_node":child,"parent_node":node['child_node'],"cost_to_go":g,"heurestic_cost":h})]);
                    
            imgct+=1
            if self.save_simulation and imgct%50==0:
                cl=np.asarray([*closed_list])           
                plt.plot(cl[:,0],cl[:,1],'.b')
                op=np.asarray([*open_list_map])
                plt.plot(op[:,0],op[:,1],'.g')
              
                plt.pause(0.000001)
                plt.savefig('./frames/astar_'+str(imgct))