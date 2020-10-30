from __future__ import print_function
import os
import time


from networkx.classes import graph
from Nodemap import *
import heapq
from itertools import count
import networkx as nx
from ReadFile import *
from collections import deque
def ReadFile():
    edge = np.loadtxt("roadNet-PA.txt", dtype="int32")
    array = np.array(edge)
    return array



def ReadHospitalFile(fileName):
    file = open("%s.txt"% fileName) 
    firstline= file.readline().strip()
    numberOfHospitals = firstline[2:]
    lines_skip_first = file.readlines()[0:]
    Hospitals =[]
    for line in lines_skip_first:
        Hospitals.append(int(line.rstrip()))
    print("hospital list",Hospitals) 
    return Hospitals
    file.close()

#not in use
#def DFS(G, origin, destination):
 #   #create empty path list to store our DFS search path thus far
  #  path_list = [[origin]]
#
    #while path list is not empty
 #   while path_list:
        #pop last path out from the path list to explore
   #     path = path_list.pop()
        #if last node in this path is the destination value/node, correct path is found
    #    last_node = path[-1] #slice path list -1 to get last element in path list
     #   if last_node ==destination:
      #      return path
        
        #else if not, continue DFS by adding new paths.
        #we do this by adding the neighbours of the last node which are not already on the list. 
       # else: 
        #    for node in G[last_node]:
         #       if node not in path:
                    # add a new path, where first element is path and second element is neighbouring node
          #          new_path = path + [node]
                    # add the new path to the total path list / tree
           #         path_list.append(new_path)
   # print("no path found")
    
#get distance to nearest hospital
#initialize 3 lists to store important values to store from processing the nodes. 
#such as distance from given hospital, visited status, as well as nearest node. 
dist =[]
visited =[]
nearHospital =[]
for i in range(1000000):
    dist.append(0)
    visited.append(0)
    nearHospital.append(0)

def multiBFS(G,hospitalQueue):
    global dist
    global visited
    # use BFS to traverse entire graph from EVERY hospital to every node.
    # saving resulting distance to 
    while(hospitalQueue):
        k = hospitalQueue.pop(0)
        #push adjacent unvisited nodes with distance from current source = node distance +1
        #traverse each adjacent neighbour , if not yet marked as visited. 
        for node in G[k]:
            if(visited[node]==0):
                #add current node to hospital queue ( BFS traversal step )
                hospitalQueue.append(node)
                #set distance of current node from source to current node distance + 1. simple increment (process current node, setting value of distance to this node (k) distance + 1 )
                #if visited by this hospital, return hospital
                nearHospital[node] = k
                #building distance from per node to hospital. 
                dist[node] = dist[k]+1
                
                #mark node as traversed.
                visited[node]=True
#pre-process step. add all source nodes, hospital to queue
def nearestHospital(G,n,sources,s):
    #queue for BFS
    q =[]
    #npGraph = nx.convert_matrix.from_numpy_array(nparray)
    global dist
    global visited
    for all in G:
        G.nodes[all]['isHospital'] =False
    #nx.set_node_attributes(G, False, "isHospital") extra
      #  G.nodes[all]['isHospital'] =False
   # attrs = {2:{"isHospital": True}, 3:{"isHospital": True},5:{"isHospital": True}, 7:{"isHospital": True}} extra
    ##nx.set_node_attributes(G,attrs)
    # mark source vertices as visited and add to queue
    
    for all in range(0,s):
        q.append(sources[all])
        G.nodes[sources[all]]['isHospital'] = True
        visited[sources[all]] = True
        
   # multiBFS(G,q)
    #print( G.nodes.data('isHospital'))
    temp= multi_source_BFS(G,q)
    '''
    #print(temp)
    for node in temp:
        PrintGraph(G, node)
        '''
'''
    #print out results of stored distance values from traversal. 
    i =1
    for allDist in range(1,n):
        print("distance of node ",i, " is ",dist[allDist],"from nearest hospital",nearHospital[allDist])
        i=i+1
        '''
    

def multi_source_BFS(g, queue):
        
    visited = {}
    start = time.time()
    temp_stored_paths = []
    while len(queue) != 0:
        # temp_stored_paths = []
        path = queue.pop(0)
        if not str(path).isnumeric():
            node = path[-1]
        else:
            node = int(path)
            path = [int(path)]
            visited[node] = True
#         print(node)
        for adjacent in g[node]:
            # print(g.graph[node][0])
            #and g.graph[adjacent][1][0] == False

            #REFERENCE                      G.nodes[sources[all]]['isHospital']
            if adjacent not in visited and (g.nodes[adjacent]['isHospital'] ==False):
                visited[adjacent] = True
                new_path = list(path)
                new_path.append(adjacent)
                # print(new_path)
                # store into variable
                temp_stored_paths.append(list(reversed(new_path)))
                queue.append(new_path)
    
        # print(temp_stored_paths)        
        # print        
#         with open("outputs1.txt", "a+") as text_file:
#            for i in temp_stored_paths:
#                text_file.write("Node: " + str(i[0]) + "  distance:  " + str(len(i) - 1) +\
#                                " Nearest hospital nodes: " + str(i[-1]) + " Path: " + str(i))
#                text_file.write('\n')
                                
        # del temp_stored_paths
    print('Search completed sucessfully, time elapsed: ', time.time()-start, 'seconds')
    print('Writing to text file now...')
    if os.path.exists("outputs1.txt"):
        os.remove("outputs1.txt")
    with open("outputs1.txt", "a+") as text_file:
        for i in temp_stored_paths:
                text_file.write("Node: " + str(i[0]) + "  distance:  " + str(len(i) - 1) +\
                                " Nearest hospital nodes: " + str(i[-1]) + " Path: " + str(i))
                text_file.write('\n')
    # REFERENCE    if adjacent not in visited and (g.nodes[adjacent]['isHospital'] ==False):
        for node in g.graph:
            if (g.nodes[node]['isHospital'] ==True):
                text_file.write("Node: " + str(node) + "  distance:  " + str(0) + \
                                " Nearest hospital nodes: " + str(node) + " Path: " + str([node]))
                text_file.write('\n')
            elif node not in visited:
                text_file.write("Node: " + str(node) + "  has no path to a hospital\n")    
                
    print('Writing completed')
    return temp_stored_paths




def display(networkmap):
    sources =[]
    sources = Hospitals
    S = len(Hospitals)
    #nparray = ReadFile()

    #nxgraph2 = nx.read_adjlist("test.adjlist")
    nearestHospital(networkmap,getNumNodes(),sources,S)
    PrintGraph(networkmap,sources)
    
    
'''
    print(" working BFS to compare with")
    start = getStart()
    end =[]
    end = Hospitals
    
    #fix edge case , crash when out of range 
    combinetext = []
    for node in range(1,getNumNodes()): #loop for every node
        if(node in end): #if node is a hospital, skip
            print(node,"skipped , is a hospital \n")
            continue
        else:
            start = node
            path = BFS(networkmap,start, end)
            print(start,end)
            if(path!=None): #if path is not empty, print details
                print("Nearest Hospital is:",path[-1])
                print("Shortest distance:",len(path), "\nShortest path is:",path, "\n")
                combinetext.extend(["start node: ", node,"Distance to nearest hospital: ",len(path), "Shortest path: ", path, "Nearest hospital: ", path[-1], "\n"])
                np.savetxt("BFSoutput.txt", combinetext,delimiter=" ", fmt="%s")
            # edges = ConvertNodeToEdge(path)
            # PrintGraph(networkmap, node)
            else: #if path empty, skip because it means node is isolated
                print("skipped, source node not connected to graph \n")
'''
# run thorugh all node , then from all nodes, run bfs to find closest hospital.
def BFS(G,origin,destination):
    #handle edge case where origin = destination, that is, origin = hospital
    if origin == destination:
        print("origin same as destination, skipped")
        return [[origin]]
    path_list = [[origin]]
    while path_list:
        path = path_list.pop(0)
        # popping from front of list, (queue functionality)
        last_node = path[-1]
        if last_node in destination: #if node is a hospital, return
            return path
        else:                        #else, add path + current node to list of paths
            for node in G[last_node]:
                if node not in path:
                    new_path = path +[node]
                    path_list.append(new_path)
    print("no path found")

def BFS_Top(G,origin,destination,k):
    incrementer =1
    while(incrementer<=k):
        path = BFS(G,origin,destination) 
        if (path != None): #if there is a returned path to nearest hospital
            print("Top Hospital",incrementer,"is",path[-1]) #find top hospital
        else:
            print("There is no nearest hospital")
        incrementer+=1
        destination.remove(path[-1]) #Remove top hospital and rerun
    
def BFSDisplay():
    #nparray = ReadFile()
    networkmap = GenerateNetworkMap()
    #print(networkmap) What is this? nothing changes if we comment out ##########################################################
    start = getStart()
    end =[]
    end = Hospitals
    
    #fix edge case , crash when out of range 
    combinetext = []
    for node in range(1,getNumNodes()): #loop for every node
        if(node in end): #if node is a hospital, skip
            print(node,"skipped , is a hospital \n")
            continue
        else:
            start = node
            path = BFS(networkmap,start, end)
            print(start,end)
            if(path!=None): #if path is not empty, print details
                print("Nearest Hospital is:",path[-1])
                print("Shortest distance:",len(path), "\nShortest path is:",path, "\n")
                combinetext.extend(["start node: ", node,"Distance to nearest hospital: ",len(path), "Shortest path: ", path, "Nearest hospital: ", path[-1], "\n"])
                np.savetxt("BFSoutput.txt", combinetext,delimiter=" ", fmt="%s")
            #edges = ConvertNodeToEdge(path)
            #PrintGraph(networkmap, node)
            else: #if path empty, skip because it means node is isolated
                print("skipped, source node not connected to graph \n")
                
def BFSTopDisplay(networkmap,k):
    #nparray = ReadFile()
    networkmap = GenerateNetworkMap()
    #print(networkmap) What is this? nothing changes if we comment out ##########################################################
    start = getStart()
    end =[]
    end = Hospitals
    
    #fix edge case , crash when out of range 
    combinetext = []
    print(getNumNodes())
    for node in range(getNumNodes()): #loop for every node
        DupHospital= end.copy()
        #DupHospital = Hospitals
        for i in range(1,k+1):
            if(node in Hospitals): #if node is a hospital, skip
                if (i == 1):
                    print(node,"skipped , is a hospital \n") #print once only
                continue
            else:
                start = node
                path = BFS(networkmap,start, DupHospital)
                print(start+1,end, DupHospital)
                if(path!=None): #if path is not empty, print details
                    print("Nearest Hospital", i, "is:",path[-1])
                    print("Shortest distance:",len(path), "\nShortest path is:",path, "\n")
                    combinetext.extend(["start node: ", node,"Distance to nearest hospital: ",len(path), "Shortest path: ", path, "Nearest hospital: ", path[-1], "\n"])
                    np.savetxt("BFSoutput.txt", combinetext,delimiter=" ", fmt="%s")
                    DupHospital.remove(path[-1])
                # edges = ConvertNodeToEdge(path)
                # PrintGraph(networkmap, node)
                else: #if path empty, skip because it means node is isolated
                    print("skipped, source node not connected to graph \n")
                    break
                
            
    
if __name__=="__main__":
    #push in nparray for bigfile.

    #
    #1A 1B running BFS for every node.
    #
    #nparray = ReadFile()
    #generate random 
   # BFSDisplay()
    
    Hospitals =ReadHospitalFile("Hospital")
    networkmap = GenerateNetworkMap()
    start = getStart()
    end =[]

    ######NOTE: every generated node map is different for (a)&(b) / (c) / (d)!!
    print("Part (a) & (b)")
    BFSTopDisplay(networkmap,1)
    print("Part (c)")
    BFSTopDisplay(networkmap,2)
    print("Part (d)")
    k = int(input("Enter value of k for k nearest hospitals:"))
    BFSTopDisplay(networkmap,k)
    
    #Multi source BFS Part B with hospital as attribute ( for random graph only )
    print("improved B, using multi source BFS")
    temp =display(networkmap)
    print(temp)

    #Refer to MultiSourceBFS for implementation that can run the bigfile
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