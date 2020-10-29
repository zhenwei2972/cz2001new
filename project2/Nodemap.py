import networkx as nx
import matplotlib.pyplot as plt
import random
#from DFS import *
start = 0
hospital = 0

def GenerateNetworkMap():
    global nodes
    nodes = random.randrange(10,20)
    edges = random.randrange(20,30)
    global a
    a = nx.gnm_random_graph(nodes, edges,random.randrange(5))
    global hospital
    hospital = random.randrange(len(list(a)))
    while True:
        global start
        start = random.randrange(len(list(a)))
        if hospital != start:
            break
    return a

def getNumNodes():
    return nodes
def getStart():
    return start
def getHospital():
    return random.randrange(len(list(a)))

counter = 1


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

def PrintGraph(networkgraph, start):
    color_map = []
    color_edge = []
    global counter
    
    Hospitals =ReadHospitalFile("Hospital")
    for node in networkgraph:
        if node in Hospitals:
            color_map.append('red')
        elif node == start:
            color_map.append('blue')
            counter += 1
        else: 
            color_map.append('green')
    
            
    #print(color_edge)
    nx.draw(networkgraph, node_color=color_map ,with_labels=True)
 
    plt.show()

def PrintGraphSimple(networkgraph, edges):
    color_map = []
    color_edge = []
    for node in networkgraph:
        if node == getHospital():
            color_map.append('red')
        elif node == getStart():
            color_map.append('blue')
        else: 
            color_map.append('green')
            
    #print(color_edge)
    nx.draw(networkgraph, node_color=color_map, edge_color=color_edge ,with_labels=True)
 
    plt.show()
def ConvertNodeToEdge(path):
    path = SortnConvert(path)
    edges = []
    for i in range(0,len(path)-1):
        single_edge = (path[i],path[i+1])
        edges.append(single_edge)
    return edges
def SortnConvert(path):
    # selection sort 
    for i in range(len(path)-1):  
        min_id = i 
        for j in range(i+1, len(path)): 
            if path[min_id] > path[j]: 
                min_id = j         
        path[i], path[min_id] = path[min_id], path[i]
    return path
