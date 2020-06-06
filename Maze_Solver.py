
import matplotlib.pyplot as plt
import Maze
import A_star 
import timeit 
import numpy as np

        
def get_astar_path(sx,sy,ex,ey,mazeX,mazeY):
    

    obs,arr=Maze.creat_map([sx,sy],[ex,ey],mazeX,mazeY)
    plt.plot(ex,ey,'xm')
    plt.plot(sx,sy,'xm')

    path_planner=A_star.A_Star(obs,[sx,sy],[ex, ey])

    # Plot Mesh
    plt.pcolormesh(arr)
    plt.axes().set_aspect('equal') #set the x and y axes to the same scale
    plt.xticks([]) # remove the tick marks by setting to an empty list
    plt.yticks([]) # remove the tick marks by setting to an empty list
    plt.axes().invert_yaxis() #invert the y-axis so the first row of data is at the top

    start_time=timeit.default_timer()
    path=path_planner.A_Star() 

    # If Path found plot the path
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
    maze_sizeX=input("Enter Maze X size. Odd values allowed: ")
    maze_sizeY=input("Enter Maze Y size. Odd values allowed: ")
    maze_size=np.array([int(maze_sizeX),int(maze_sizeY)])

    assert maze_size.shape==(2,),"Incorrect Shape"
    assert maze_size[0]%2 ==1, "Incorrect size value"
    assert maze_size[1]%2 ==1, "Incorrect size value"

    startX=input("Enter start X. Even values allowed: ")
    startY=input("Enter start Y. Even values allowed: ")
    start_node=np.array([int(startX),int(startY)])

    assert start_node.shape==(2,),"Incorrect Shape"
    assert start_node[0]%2 ==0, "Incorrect size value"
    assert start_node[1]%2 ==0, "Incorrect size value"

    endX=input("Enter end X. Even values allowed: ")
    endY=input("Enter end Y. Even values allowed: ")
    end_node=np.array([int(endX),int(endY)])

    assert end_node.shape==(2,),"Incorrect Shape"
    assert end_node[0]%2 ==0, "Incorrect size value"
    assert end_node[1]%2 ==0, "Incorrect size value"


    get_astar_path(sx=start_node[0],sy=start_node[1],ex=end_node[0],ey=end_node[1],mazeX=maze_size[0],mazeY=maze_size[1])  