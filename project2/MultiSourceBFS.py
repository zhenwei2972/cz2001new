from __future__ import print_function
import os
import time

import pandas as pd
from collections import defaultdict
from networkx.classes import graph
from Nodemap import *
import heapq
from itertools import count
import networkx as nx
from ReadFile import *
from collections import deque


# Own Graph, multi source bfs that works on bigfile
# 
          
            
#graph structure
# implement adjacency list representation with hospital = True/False as an extra element for each node
class Graph:  
    # Constructor 
    def __init__(self): 
        # default dictionary to store graph 
        self.graph = defaultdict(list) 
        self.max_key = None
        self.min_key = None
        self.result = defaultdict(list)
        
    # function to add an edge to graph. Input: int(source), int(destination)
    def addEdge(self,u,v): 
        if self.graph[u] == []:
            self.graph[u].append([v])
        else:
            self.graph[u][0].append(v)
    
    # function to add a dummy edge (node,node) if a valid node has no edge    
    # purpose: to identify 2 special cases 
    # case 1: if a node is in the map, it has no edge and it is a hospital
    # case 2: if a node is in the map, it has no edge and it is not a hospital   
    # assumption: After adding in all given edges from the dataset,
    # all nodes(integer numbers) between the current smallest and the largest node numbers are valid nodes
    # eg. Given (1,2) (2,1) (3,5) (5,3), then 4 is also a valid node with no edge
    def fill_nodes_without_edge(self):
        max_key, dummy_var = random.choice(list(self.graph.items()))
        min_key, dummy_var = random.choice(list(self.graph.items()))
        for key in self.graph:
            if key > max_key:
                max_key = key
            if key < min_key:
                min_key = key
            for value in self.graph[key][0]:
                if value > max_key:
                    max_key = value
                if value < min_key:
                    min_key = value                
        self.max_key = max_key
        self.min_key = min_key
        all_nodes = [*range(min_key, max_key+1, 1)] 
        for node in all_nodes:
            if self.graph[node] == []: # if a node has no edge
                self.graph[node].append([node]) 
    
    # function to add a true or false variable to each entry in the graph. True only if a node is a hospital node.
    # Input: a list of hospital nodes eg. [1,3,5,7,9]   
    def addHospital(self, h_list): 
        # initialize all nodes with hospital = False
        for key in self.graph:
            self.graph[key].append([False])
            
        invalid_hospitals = []                
        for h in h_list:
            # ensure all hospital entries are valid 
            if int(h)>self.max_key or int(h)<self.min_key:
                invalid_hospitals.append(int(h))
            else:
                self.graph[int(h)][1][0] = True

        if invalid_hospitals == []:
            print('all hospitals added sucessfully')
        else:
            print(len(invalid_hospitals), 'hospitals not added due to invalid node number')
        return invalid_hospitals
    
    def BFS(self, start):
        # maintain a queue of paths
        queue = []
        # push the first path into the queue
        queue.append([start])

        # create a dict of all keys, initialize all key value to False
        visited = {}        
        
        while queue:
            # get the first path from the queue
            path = queue.pop(0)
            # get the last node from the path
            node = path[-1]
            # set dictionary[adjacent] = True
            visited[node] = True
            # if path found in current node
            if self.graph[node][1][0]:
                return path, len(path)-1
                        
            # for unvisited adjacent nodes, construct a new path and push it into the queue
            for adjacent in self.graph[node][0]:
                # if a adjacency node is already proven to be a dead end
                if self.result[adjacent] != [] and self.result[adjacent][1] == -1:
                    visited[adjacent] = True
                    continue
                if adjacent in visited:
                    continue
                new_path = list(path)
                new_path.append(adjacent)               
                queue.append(new_path)   

        return [],-1      
    
    def run_BFS(self):
        #total_nodes = len(self.graph) 
        #tau = 0.0001
        #mul = tau
        #percentage_done = []
        #time_stamp = []
        #start = time.time()
        for i, node in enumerate(self.graph):
            if self.result[node] == []: # if a node hasn't find the path yet
                path, path_length = self.BFS(node)
                if path_length == -1: # if no path exist
                    self.result[node].append([])
                    self.result[node].append(-1)
                else:
                    # update shortest path for all nodes along this sucessfully searched path
                    for j in range(path_length+1): 
                        if self.result[path[j]] != []:
                            break
                        self.result[path[j]].append(path[j:])
                        self.result[path[j]].append(path_length-j)   
                    
            # tracking progress 
            #if i+1 == int(mul*total_nodes):
            #    print('{0:.2f}% nodes scanned, time elapsed: {1:.0f} seconds'.format(mul*100, time.time()-start))
            #    percentage_done.append(mul*100)
            #    time_stamp.append(time.time()-start)
            #    mul += tau
                

        # write to output file
        if os.path.exists("outputs_a_b_naive_bfs.txt"):
            os.remove("outputs_a_b_naive_bfs.txt")
        #print(self.result)
        with open("outputs_a_b_naive_bfs.txt", "a+") as text_file:
            for key in self.result:
                if self.result[key][0] == []:                    
                    text_file.write("Node: " + str(key) + "  has no path to a hospital")
                else:
                    text_file.write("Node: " + str(key) + "  distance:  " + str(self.result[key][1]) +\
                    " Nearest hospital nodes: " + str(self.result[key][0][-1]) + " Path: " +\
                    str(self.result[key][0]))
                text_file.write('\n') 

def multi_source_BFS2(g, queue):   
    visited = {}
    start = time.time()
    temp_stored_paths = []
    while queue:
        path = queue.pop(0)
        if not str(path).isnumeric():
            node = path[-1]
        else:
            node = int(path)
            path = [int(path)]
            visited[node] = True
        for adjacent in g.graph[node][0]:
            if adjacent not in visited and g.graph[adjacent][1][0] == False:
                visited[adjacent] = True
                new_path = list(path)
                new_path.append(adjacent)
                temp_stored_paths.append(list(reversed(new_path)))
                queue.append(new_path)

    print('Search completed sucessfully, time elapsed: ', time.time()-start, 'seconds')
    print('Writing to text file now...')
    if os.path.exists("outputs_a_b_ms_bfs.txt"):
        os.remove("outputs_a_b_ms_bfs.txt")
    with open("outputs_a_b_ms_bfs.txt", "a+") as text_file:
        for i in temp_stored_paths:
                text_file.write("Node: " + str(i[0]) + "  distance:  " + str(len(i) - 1) +\
                                " Nearest hospital nodes: " + str(i[-1]) + " Path: " + str(i))
                text_file.write('\n')
        for node in g.graph:
            if g.graph[node][1][0]:
                text_file.write("Node: " + str(node) + "  distance:  " + str(0) + \
                                " Nearest hospital nodes: " + str(node) + " Path: " + str([node]))
                text_file.write('\n')
            elif node not in visited:
                text_file.write("Node: " + str(node) + "  has no path to a hospital\n")    
    print('Writing completed')
    return temp_stored_paths

if __name__=="__main__":

    
    # read in real map data 
    path = 'roadNet-CA.txt.gz' #

    road_df = pd.read_csv(path, compression='gzip', header=0, sep=',',\
                        quotechar='"', error_bad_lines=False)
    s1 = road_df.iloc[1,0]
    s2 = s1.split(':')[1]
    nodes = int(s2.split(' ')[1])
    edges = int(s1.split('Edges:')[1].replace(" ",""))
    road_df = road_df.iloc[:, 0][3:]
    print(s1)  
    #read in hospital
    hospital_file = open('Hospital.txt', 'r') 
    Lines = hospital_file.readlines() 
    hospital = []
    for line in Lines: 
        hospital.append(line.strip())
    
        # implement adjacency list representation with hospital = True/False as an extra element for each node
    g = Graph() 
    start = time.time()

    # add edges
    for i in range(len(road_df)):
        edge = road_df.iloc[i].split('\t')
        u = int(edge[0])
        v = int(edge[1])
        g.addEdge(u, v)
    g.fill_nodes_without_edge()
    print('all edges added sucessfully')

    # check for invalid hospitals, add hospitals   
    invalid_hospitals = g.addHospital(hospital[1:])

    print('pre-processing time: ', time.time() - start)

    #run multi source bfs
    temp_stored_paths = multi_source_BFS2(g, hospital[1:]) # multi-source BFS
    
#get count of hospitals from text file
#countH = int(hospital[0].replace('#', ''))
#print('Hospital: ',countH)
    #push in nparray for bigfile.

    #
    #1A 1B running BFS for every node.
    #
    #nparray = ReadFile()
    #generate random 
   # BFSDisplay()
'''
    Hospitals =ReadHospitalFile("Hospital")
    networkmap = GenerateNetworkMap()
    #print(networkmap) What is this? nothing changes if we comment out ########################################
    start = getStart()
    end =[]
    '''
    #prototype , probably not correct way to do it. 

    # working code commented out.

    ######NOTE: every generated node map is different for (a)&(b) / (c) / (d)!!
    #print("Part (a) & (b)")
    #BFSTopDisplay(1)
    #print("Part (c)")
    #BFSTopDisplay(2)
    #print("Part (d)")
    #k = int(input("Enter value of k for k nearest hospitals:"))
    #BFSTopDisplay(k)
    
    #working multi source based on our own code
    #testing code
    #temp =display()
    #print(temp)
'''

    combinetext = []
    for y in range(getNumNodes()):
        if(y in end):
            print(y,"skipping , is a hospital")
            #if hospital skip
            continue
        else:
            start =y
            path = dijsktra(networkmap,start, end)
            print(start,end)
            if(path!=None):
                print("Nearest Hospital is",path[-1])
                print("Shortest path is ",path)
                combinetext.append("start node")
                combinetext.append(y)
                combinetext.append(path)
                combinetext.append("hospital")
                combinetext.append(path[-1])
                combinetext.append("\n")
                np.savetxt("BFSoutput.txt", combinetext,delimiter=" ", fmt="%s")
                edges = ConvertNodeToEdge(path)
                PrintGraph(networkmap, y)
            else:
                print("skipped, source node not connected to graph")
                '''



    #blue is start. 
    #red is end 
   # PrintGraphSimple(networkmap, edges)