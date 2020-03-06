import numpy as np 
import cv2
import sys
import heapq
import argparse
import matplotlib.pyplot as plt
class Node:
    
    def __init__(self,start_coord,goal_coord,radius,clearance):
        
        self.start_coord = start_coord # Start (x,y) coordinates
        self.goal_coord = goal_coord # End (x,y) coordinates
        self.radius = radius # Radius of the robot. For a point robot its 0 and for a rigid robot the user can specify any value
        self.clearance = clearance # The distance of the robot from the obstacle which should be maintained at all times
        self.columns = 300 # Columns in the given Map
        self.rows = 200 # Rows in the given Map
     
    # Calculate area of a triangle is a helper function as we have subdivided the figures into a number of smaller triangles 
    def calAreaOfTriangle(self,a_x,a_y,b_x,b_y,c_x,c_y):
        " Returns the absolute value of the area as it cannot be negative"
        #return np.abs(((a_x)*((b_y)-(c_y)) + (b_x)*((c_y)-(a_y))+(c_x)((a_y)-(b_y))/2))
        return np.abs((a_x*(b_y-c_y) + b_x*(c_y-a_y) + c_x*(a_y - b_y))/2)
    
    # Various conditions for detecting obstacles in the given map
    # Following numpy conventions but OpenCV Indexing
    def ObstacleDetection(self,x_row,y_column):
        
        radiusClearanceSum = self.radius + self.clearance
        "Condition for whether the robot is nearing a circle obstacle"
        circdist = (y_column-225)**2 + (x_row - 150)**2 - (25 + radiusClearanceSum) ** 2
        
        "Condition for whether the robot is nearing an elliptical obstacle"
        ellipticalDist = (y_column-150)**2 / (40 + radiusClearanceSum)**2 + (x_row - 100)**2 / (20 + radiusClearanceSum)**2 - 1
        
        "Condition for whether the robot is nearing a rhombus obstacle"
        #We use the concept of finding a point inside the rectangle. If we have a point inside the triangle then we calculate three
        #areas by connecting the point to each vertex of the triangle and if that is equal to the whole triangle area the point lies
        #inside the triangle
        " First Triangle of the rhombus"
        bigTriangleArea1 = self.calAreaOfTriangle(10-((1.4142)*(radiusClearanceSum)),225,25,200-(1.4142*(radiusClearanceSum)),40+(1.4142*(radiusClearanceSum)),225)
        smallTriangleArea1 = self.calAreaOfTriangle(x_row,y_column,25,200-1.4142*(radiusClearanceSum),40+1.4142*(radiusClearanceSum),225)
        smallTriangleArea2 = self.calAreaOfTriangle(x_row,y_column,25,200-1.4142*(radiusClearanceSum),10-1.4142*(radiusClearanceSum),225)
        smallTriangleArea3 = self.calAreaOfTriangle(x_row,y_column,10-1.4142*(radiusClearanceSum),225,40+1.4142*(radiusClearanceSum),225)
        
        distRobot1 = (smallTriangleArea1 + smallTriangleArea2 + smallTriangleArea3) - (bigTriangleArea1)
        
        if(distRobot1 < 1e-5):
            distRobot1 = 0
        
        "Second triangle of the rhombus"
        bigTriangleArea2 = self.calAreaOfTriangle(10-np.sqrt(2)*(radiusClearanceSum),225,25,250+np.sqrt(2)*(radiusClearanceSum),40+np.sqrt(2)*(radiusClearanceSum),225)
        smallTriangleArea2_1 = self.calAreaOfTriangle(x_row,y_column,25,250+np.sqrt(2)*(radiusClearanceSum),40+np.sqrt(2)*(radiusClearanceSum),225)
        smallTriangleArea2_2 = self.calAreaOfTriangle(x_row,y_column,25,250+np.sqrt(2)*(radiusClearanceSum),10-np.sqrt(2)*(radiusClearanceSum),225)
        smallTriangleArea2_3 = self.calAreaOfTriangle(x_row,y_column,10-np.sqrt(2)*(radiusClearanceSum),225,40+np.sqrt(2)*(radiusClearanceSum),225)
        
        distRobot2 = (smallTriangleArea2_1 + smallTriangleArea2_2 + smallTriangleArea2_3) - (bigTriangleArea2)
        
        if(distRobot2 < 1e-5):
            distRobot2 = 0
            
        "Condition for whether the robot is nearing the RECTANGLE on an ANGLE obstacle"
        "First triangle of the angled rectangle"
        bigTriangleArea1 = self.calAreaOfTriangle(30-np.sqrt(2)*(radiusClearanceSum),95,67.5,30.05-np.sqrt(2)*(radiusClearanceSum),76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        smallTriangleArea1 = self.calAreaOfTriangle(x_row,y_column,30-np.sqrt(2)*(radiusClearanceSum),95,67.5,30.05-np.sqrt(2)*(radiusClearanceSum))
        smallTriangleArea2 = self.calAreaOfTriangle(x_row,y_column,67.5,30.05-np.sqrt(2)*(radiusClearanceSum),76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        smallTriangleArea3 = self.calAreaOfTriangle(x_row,y_column,30-np.sqrt(2)*(radiusClearanceSum),95,76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        
        distRobot3 = (smallTriangleArea1 + smallTriangleArea2 + smallTriangleArea3) - (bigTriangleArea1)
        if(distRobot3 < 1e-5):
            distRobot3 = 0
        
        "Second triangle of the angled rectangle"
        bigTriangleArea2 = self.calAreaOfTriangle(30-np.sqrt(2)*(radiusClearanceSum),95,38.65,100+np.sqrt(2)*(radiusClearanceSum),76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        smallTriangleArea2_1 = self.calAreaOfTriangle(x_row,y_column,30-np.sqrt(2)*(radiusClearanceSum),95,38.65,100+np.sqrt(2)*(radiusClearanceSum))
        smallTriangleArea2_2 = self.calAreaOfTriangle(x_row,y_column,38.65,100+np.sqrt(2)*(radiusClearanceSum),76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        smallTriangleArea2_3 = self.calAreaOfTriangle(x_row,y_column,30-np.sqrt(2)*(radiusClearanceSum),95,76.15+np.sqrt(2)*(radiusClearanceSum),35.5)
        
        distRobot4 = (smallTriangleArea2_1 + smallTriangleArea2_2 + smallTriangleArea2_3) - (bigTriangleArea2)
        
        if(distRobot4 < 1e-5):
            distRobot4 = 0
        
        "Condition for whether the robot is nearing the polygon obstacle"
        " Can be divided into a square and an triangle"
        " Evaluating the square first we get"
        bigTriangleArea1 = self.calAreaOfTriangle(120-np.sqrt(2)*(radiusClearanceSum),75,150-np.sqrt(2)*(radiusClearanceSum),50,185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        smallTriangleArea1 = self.calAreaOfTriangle(x_row,y_column,150-np.sqrt(2)*(radiusClearanceSum),50,185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        smallTriangleArea2 = self.calAreaOfTriangle(x_row,y_column,120-np.sqrt(2)*(radiusClearanceSum),75,150-np.sqrt(2)*(radiusClearanceSum),50)
        smallTriangleArea3 = self.calAreaOfTriangle(x_row,y_column,120-np.sqrt(2)*(radiusClearanceSum),75,185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        
        distRobot5 = (smallTriangleArea1 + smallTriangleArea2 + smallTriangleArea3) - (bigTriangleArea1)
        
        if(distRobot5 < 1e-5):
            distRobot5 = 0
            
        " Second Triangle part of the square enclosed inside the polygon"
        bigTriangleArea2 = self.calAreaOfTriangle(120-np.sqrt(2)*(radiusClearanceSum),75,150,100+np.sqrt(2)*(radiusClearanceSum),185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        smallTriangleArea2_1 = self.calAreaOfTriangle(x_row,y_column,150,100+np.sqrt(2)*(radiusClearanceSum),185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        smallTriangleArea2_2 = self.calAreaOfTriangle(x_row,y_column,120-np.sqrt(2)*(radiusClearanceSum),75,150,100+np.sqrt(2)*(radiusClearanceSum))
        smallTriangleArea2_3 = self.calAreaOfTriangle(x_row,y_column,120-np.sqrt(2)*(radiusClearanceSum),75,185+(radiusClearanceSum),75+(0.714*radiusClearanceSum))
        
        distRobot6 = (smallTriangleArea2_1 + smallTriangleArea2_2 + smallTriangleArea2_3) - (bigTriangleArea2)
        
        if(distRobot6 < 1e-5):
            distRobot6 = 0

        "EVALUATING THE TRIANGLE PART OF THE NON CONVEX POLYGON"
        bigTriangleArea1 = self.calAreaOfTriangle(120-(2.62*radiusClearanceSum),20-(1.205*radiusClearanceSum),150-(np.sqrt(2)*radiusClearanceSum),50,185+radiusClearanceSum,25-(radiusClearanceSum*1.081)) 
        smallTriangleArea1 = self.calAreaOfTriangle(x_row,y_column,150-(np.sqrt(2)*radiusClearanceSum),50,185+radiusClearanceSum,25-(radiusClearanceSum*1.081))
        smallTriangleArea2 = self.calAreaOfTriangle(x_row,y_column,120-(2.62*radiusClearanceSum),20-(1.205*radiusClearanceSum),185+radiusClearanceSum,25-(radiusClearanceSum*1.081))
        smallTriangleArea3 = self.calAreaOfTriangle(x_row,y_column,120-(2.62*radiusClearanceSum),20-(1.205*radiusClearanceSum),150-(np.sqrt(2)*radiusClearanceSum),50)
        
        distRobot7 = (smallTriangleArea1 + smallTriangleArea2 + smallTriangleArea3) - (bigTriangleArea1)
        
        if(distRobot7 < 1e-5):
            distRobot1 = 0

        bigTriangleArea2 = self.calAreaOfTriangle(150-(np.sqrt(2)*radiusClearanceSum),50,185+radiusClearanceSum,25-(1.081*radiusClearanceSum),185+radiusClearanceSum,75+(radiusClearanceSum*0.714))
        smallTriangleArea2_1 = self.calAreaOfTriangle(x_row,y_column,150-(np.sqrt(2)*radiusClearanceSum),50,185+radiusClearanceSum,25-(1.081*radiusClearanceSum))
        smallTriangleArea2_2 = self.calAreaOfTriangle(x_row,y_column,150-(np.sqrt(2)*radiusClearanceSum),50,185+radiusClearanceSum,75+(radiusClearanceSum*0.714))
        smallTriangleArea2_3 = self.calAreaOfTriangle(x_row,y_column,185+radiusClearanceSum,25-(1.081*radiusClearanceSum),185+radiusClearanceSum,75+(radiusClearanceSum*0.714))

        distRobot8 = (smallTriangleArea2_1 + smallTriangleArea2_2 + smallTriangleArea2_3) - (bigTriangleArea2)
        
        if(distRobot8 < 1e-5):
            distRobot8 = 0
    
        if(circdist<=0 or ellipticalDist<=0 or distRobot1 == 0 or distRobot2 == 0 or distRobot3 == 0 or distRobot4 == 0 or distRobot5 == 0 or distRobot6 == 0 or distRobot7 == 0 or distRobot8 == 0):
            return True
        else:
            return False
        
    " Function to check if the move is valid or not"
    def validMove(self,row,column):
        if(row >=(self.radius+self.clearance) and row<(self.rows - self.clearance-self.radius) and column >= (self.radius+self.clearance) and column < (self.columns-self.clearance-self.radius)):
            return True
        else:
            return False
    
    " Functions for scanning through the nodes in the up, down, left, right, upleft, upright, downleft and downright directions"
    " As OpenCV uses the opposite conventions for labeling x and y axis the move functions will also be opposite in indicating"
    " the movement in a specific direction"
    def ActionMoveLeft(self,x_row,y_column):
        if(self.ObstacleDetection(x_row,y_column-1) == False and self.validMove(x_row,y_column-1) == True):
            return True
        else:
            return False
    def ActionMoveDown(self,x_row,y_column):
        if(self.ObstacleDetection(x_row+1,y_column) == False and self.validMove(x_row+1,y_column) == True):
            return True
        else:
            return False
    def ActionMoveRight(self,x_row,y_column):
        if(self.ObstacleDetection(x_row,y_column+1) == False and self.validMove(x_row,y_column+1) == True):
            return True
        else:
            return False
    def ActionMoveUp(self,x_row,y_column):
        if(self.ObstacleDetection(x_row-1,y_column) == False and self.validMove(x_row-1,y_column) == True):
            return True
        else:
            return False
    def ActionMoveUpLeft(self,x_row,y_column):
        if(self.ObstacleDetection(x_row-1,y_column-1) == False and self.validMove(x_row-1,y_column-1) == True):
            return True
        else:
            return False
    def ActionMoveUpRight(self,x_row,y_column):
        if(self.ObstacleDetection(x_row-1,y_column+1) == False and self.validMove(x_row-1,y_column+1) == True):
            return True
        else:
            return False
    def ActionMoveDownLeft(self,x_row,y_column):
        if(self.ObstacleDetection(x_row+1,y_column-1) == False and self.validMove(x_row+1,y_column-1) == True):
            return True
        else:
            return False
    def ActionMoveDownRight(self,x_row,y_column):
        if(self.ObstacleDetection(x_row+1,y_column+1) == False and self.validMove(x_row+1,y_column+1) == True):
            return True
        else:
            return False
        
    def dijkstra(self):
        " Will be using priority queue using pythons inbuilt heapq"
        " First we build a hapsh map to store the vertex and the corresponding distances"
        visited_nodes = {} # Store all the visited nodes
        explored_nodes = [] # set to store the explored states
        priority_queue = [] # Min-Heap
        visited_and_distance = {} # A hash map/dictionary to store the nodes and their corresponding distances from the source vertex
        prev = {} # Required for backtracking. Store the path for the predecessor
        backtrack_nodes = [] # list to store backtracked nodes
        
        " First we need to initialize the distances from the source vertex to all the other vertices with infinity"
        " Iterate through the number of rows and columns"
        for i in range(0,self.rows):
            for j in range(0,self.columns):
                
                visited_and_distance[(i,j)] = float('inf') # Set all distances to infinity
                visited_nodes[(i,j)] = False 
                prev[(i,j)] = -1
                
        # Push into the heap and initialize the source to source distance as 0
        heapq.heappush(priority_queue,(0,self.start_coord))
        visited_and_distance[self.start_coord] = 0 # Distance to itself
        
        # Dijkstra Algorithm 
        while(len(priority_queue) > 0):
            
            # First we pop from the priority queue
            dist , node = heapq.heappop(priority_queue)
            visited_nodes[node] = True
            explored_nodes.append(node)
            
            # First we check if the current node is equal to the goal node. If it is equal then exit 
            if(node[0] == self.goal_coord[0] and node[1] == self.goal_coord[1]):
                break
            
            # Now we move in all the possible directions and update the costs accordingly
            if(self.ActionMoveLeft(node[0],node[1])==True and visited_nodes[(node[0],node[1]-1)] == False and (visited_and_distance[(node[0],node[1]-1)] > visited_and_distance[node] + 1)):
                visited_and_distance[(node[0],node[1]-1)] = visited_and_distance[node] + 1
                prev[(node[0],node[1]-1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0],node[1]-1)],(node[0],node[1]-1)))
                
            if(self.ActionMoveRight(node[0],node[1])==True and visited_nodes[(node[0],node[1]+1)] == False and (visited_and_distance[(node[0],node[1]+1)] > visited_and_distance[node] + 1)):
                visited_and_distance[(node[0],node[1]+1)] = visited_and_distance[node] + 1
                prev[(node[0],node[1]+1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0],node[1]+1)],(node[0],node[1]+1)))
            
            if(self.ActionMoveDown(node[0],node[1])==True and visited_nodes[(node[0]+1,node[1])] == False and (visited_and_distance[(node[0]+1,node[1])] > visited_and_distance[node] + 1)):
                visited_and_distance[(node[0]+1,node[1])] = visited_and_distance[node] + 1
                prev[(node[0]+1,node[1])] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]+1,node[1])],(node[0]+1,node[1])))
                
            if(self.ActionMoveUp(node[0],node[1])==True and visited_nodes[(node[0]-1,node[1])] == False and (visited_and_distance[(node[0]-1,node[1])] > visited_and_distance[node]) + 1):
                visited_and_distance[(node[0]-1,node[1])] = visited_and_distance[node] + 1
                prev[(node[0]-1,node[1])] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]-1,node[1])],(node[0]-1,node[1])))
                
            if(self.ActionMoveUpLeft(node[0],node[1])==True and visited_nodes[(node[0]-1,node[1]-1)] == False and (visited_and_distance[(node[0]-1,node[1]-1)] > visited_and_distance[node] + np.sqrt(2))):
                visited_and_distance[(node[0]-1,node[1]-1)] = visited_and_distance[node] + np.sqrt(2)
                prev[(node[0]-1,node[1]-1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]-1,node[1]-1)],(node[0]-1,node[1]-1)))
                
            if(self.ActionMoveUpRight(node[0],node[1])==True and visited_nodes[(node[0]-1,node[1]+1)] == False and (visited_and_distance[(node[0]-1,node[1]+1)] > visited_and_distance[node] + np.sqrt(2))):
                visited_and_distance[(node[0]-1,node[1]+1)] = visited_and_distance[node] + np.sqrt(2)
                prev[(node[0]-1,node[1]+1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]-1,node[1]+1)],(node[0]-1,node[1]+1)))
                
            if(self.ActionMoveDownLeft(node[0],node[1])==True and visited_nodes[(node[0]+1,node[1]-1)] == False and (visited_and_distance[(node[0]+1,node[1]-1)] > visited_and_distance[node] + np.sqrt(2))):
                visited_and_distance[(node[0]+1,node[1]-1)] = visited_and_distance[node] + np.sqrt(2)
                prev[(node[0]+1,node[1]-1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]+1,node[1]-1)],(node[0]+1,node[1]-1)))
                
            if(self.ActionMoveDownRight(node[0],node[1])==True and visited_nodes[(node[0]+1,node[1]+1)] == False and (visited_and_distance[(node[0]+1,node[1]+1)] > visited_and_distance[node] + np.sqrt(2))):
                visited_and_distance[(node[0]+1,node[1]+1)] = visited_and_distance[node] + np.sqrt(2)
                prev[(node[0]+1,node[1]+1)] = node
                heapq.heappush(priority_queue,(visited_and_distance[(node[0]+1,node[1]+1)],(node[0] + 1,node[1]+1)))
                
        if(visited_and_distance[self.goal_coord] == float('inf')):
            
            # Meaning no optimal path
            return (visited_and_distance[self.goal_coord],[],explored_nodes)
        
        # BackTracking
        print("dOING")
        node = self.goal_coord
        while(prev[node] != -1):
            backtrack_nodes.append(node)
            node = prev[node]
        backtrack_nodes.append(self.start_coord)
        
        # Reversing the list so that the order is start_node-------> goal_node
        backtrack_nodes = list(reversed(backtrack_nodes))
        return(visited_and_distance[self.goal_coord], backtrack_nodes, explored_nodes)
        
    def pathAnimation(self, explored_nodes, backtrack_nodes, prev):
        print("Animating!")
        # First create a black mask frame 
        frame = np.zeros((self.rows,self.columns,3),dtype = np.uint8)
        
        # Create a VideoWriter object to save continous frame sequences to form an animation
        fourcc = cv2.VideoWriter_fourcc(*'DVIX')
        out = cv2.VideoWriter(str(prev),fourcc,20,(self.rows,self.columns)) # Output Video, codec, frames per second, size
        
        for node in explored_nodes:
            frame[int(self.rows-node[0]-1),int(node[1])] = (255,0,0)
            out.write(frame)
            
        for row in range(0,self.rows):
            for column in range(0,self.columns):
                
                # Check for black pixels for each colour channels
                if(frame[int(self.rows-row-1),int(column-1),0]==0 and frame[int(self.rows-row-1),int(column-1),1]==0 and frame[int(self.rows-row-1),int(column-1),2]==0):
                    if(self.validMove(row,column) == True and self.ObstacleDetection(row,column) == False):
                        frame[int(self.rows-row-1),int(column)] = [0,250,0]
                        out.write(frame)
        
        if(len(backtrack_nodes) > 0):
            for nodes in backtrack_nodes:
                frame[int(self.rows-nodes[0]-1),int(nodes[1])] = (0,0,255)
                out.write(frame)
                cv2.imshow('Path taken', frame)
                cv2.waitKey(5)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()
        
def main():
        
    start_coord_row = int(input("Enter the starting row coordinates of the node(1-200):  "))
    start_coord_column = int(input("Enter the starting column coordinates of the node(1-300):  ")) 
    goal_coord_row = int(input("Enter the starting goal node row coordintes(1-200):  "))
    goal_coord_column = int(input("Enter the starting goal node column coordinates(1-300):  "))
    radius = int(input("Enter the radius of the rigid robot. Enter 0 if you want a simulation for a point robot:  "))
    clearance = int(input("Enter the max clearnace you want the robot to maintain from an obstacle. Enter 0 if you want a simulation for a point robot:  "))
        
    start_coord = (start_coord_row,start_coord_column)
    goal_coord = (goal_coord_row, goal_coord_column)
        
    dijkstra = Node(start_coord,goal_coord,radius,clearance)
        
    if(dijkstra.validMove(start_coord[0],start_coord[1]) == True and dijkstra.validMove(goal_coord[0],goal_coord[1]) == True and dijkstra.ObstacleDetection(start_coord[0],start_coord[1]) == False and dijkstra.ObstacleDetection(goal_coord[0],goal_coord[1]) == False):
        
                    (distance_from_start_to_goal, backtrack_states, explored_states) = dijkstra.dijkstra()
                    print(len(backtrack_states))
                    dijkstra.pathAnimation(explored_states, backtrack_states, "./dijkstra_simulation.avi")

                    # print optimal path found or not
                    if(distance_from_start_to_goal == float('inf')):
                        print("\nNo optimal path found.")
                    else:
                        print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
    else:
        print("The entered goal node is an obstacle ")
        
if __name__ == "__main__":
    main()