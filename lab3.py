import networkx as nx
import matplotlib.pyplot as plt
from collections import defaultdict
import heapq


def dijkstra(G, start, end):

    """
    Dijkstra algorithm for creating a forwarding table using least cost path

    Parameters:
    G(netorkx graph): graph
    start: The starting node
    end:   The end node 

    Returns:
    path: the sequence of nodes label from starting point to end point
    """

    # Check if start and end nodes exist in the graph
    if start not in G.nodes() or end not in G.nodes():
        return []

    # Initialize the distance and previous node dictionaries
    dist = {node: float('inf') for node in G.nodes()}
    dist[start] = 0
    prev = {node: None for node in G.nodes()}

    # Initialize visited set and heap with start node
    visited = set()
    heap = [(0, start)]

    # Dijkstra's algorithm
    while heap:
        # Pop the smallest element from the heap
        (d, u) = heapq.heappop(heap)

        # Skip if already visited
        if u in visited:
            continue
        visited.add(u)

        # If end node reached, break the loop
        if u == end:
            break

        # Explore the neighbors of the current node
        for v in G.neighbors(u):
            # Skip if already visited
            if v in visited:
                continue

            # Calculate the tentative distance to neighbor through current node
            alt = dist[u] + G[u][v]['weight']

            # Update the distance and previous node if shorter path found
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u

                # Push the neighbor into the heap with its new distance
                heapq.heappush(heap, (alt, v))

    # Construct the shortest path from start to end node using the previous node dictionary
    path = []
    node = end
    while node is not None:
        path.insert(0, node)
        node = prev[node]

    # If the start node is not the first element in the path, it means there's no path between start and end
    if path[0] != start:
        return []
    else:
        return path
    


path = input('Enter file name:') #taking file input from the user
path = path + '.txt' #adding the extension

try:
    with open(path, "r") as f:
        #open the file and store its content in an array of strings each of which contains one line of file
        data = f.readlines()
except FileNotFoundError:
    print('FILE NOT FOUND!')

G = nx.Graph()
nodes =[] #empty list for nodes in graph
data = data[1:] # Drop the first line of data that contains the number of edges and vertices
for i in data:
    if i[0] not in nodes:
        G.add_node(i[0])
        nodes.append(i[0])
    if i[2] not in nodes:
        G.add_node(i[2])
        nodes.append(i[2])
    G.add_edge(i[0], i[2], weight= float(i[4])) # Add weighted edge to the graph

#creating forwarding table using dijkstra algorithm
for i in nodes:
    print('Forwarding Table for ' + i)
    print('*****************************************')
    for j in nodes:
        if j != i:
            link = dijkstra(G, i, j)
            print('Destination ' + j + ': ' + link[0]+' --> '+link[1])
            print('-----------------------------------------')

pos=nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, font_weight='bold')
edge_weight = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels = edge_weight)

plt.show() # Show the plot