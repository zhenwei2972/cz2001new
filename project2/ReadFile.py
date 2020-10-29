
import numpy as np
import networkx as nx
from Nodemap import *
import matplotlib.pyplot as plt
def readTxt():
    # Reading the file. "DiGraph" is telling to reading the data with node-node. "nodetype" will identify whether the node is number or string or any other type.
    g = nx.Graph()
    g = nx.read_adjlist("roadNet-PA.txt", delimiter="\t")
    nx.write_adjlist(g, "test.adjlist")
    return g
def fasterRead():
    fh = open("test.adjlist","rb")
    G = nx.read_adjlist(fh,nodetype=int)
    nx.write_gpickle(G,"file.gpickle")
    return G
def getBigGraph():
    g = nx.read_gpickle("file.gpickle")
    return g
if __name__=="__main__":
    
    g = nx.read_gpickle("file.gpickle")
    #nx.draw(g,with_labels=True)
    print(len(g))

    


