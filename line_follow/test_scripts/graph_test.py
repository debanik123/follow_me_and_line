import networkx as nx
import matplotlib.pyplot as plt

# Define the edges based on the connections you provided
edges = [
    (1,2), 
    (2,3), (2,4),
    (8,5), (5,3), (5,6), (6,7), (7,4), (7,9),
    (11, 10), (10, 6), (10, 12), 
    (10, 13)
]

G = nx.Graph()
G.add_edges_from(edges)

# Specify node positions manually
pos = {
    1: (0, 0),  # position of node 1
    2: (1, 0),  # position of node 2
    3: (2, 1),  # position of node 3
    4: (2, -1),  # position of node 4 and so on ..
    8: (3, 2),
    5: (3, 1),
    6: (3, 0),
    7: (3, -1),
    9: (3, -2),
    11: (4,1),
    10: (4,0),
    12:(4,-1),
    13: (5, 0)
}

# Draw the graph
nx.draw(G, pos, with_labels=True, node_size=700, node_color="skyblue", font_size=8, font_color="black", font_weight="bold", arrowsize=10)

# Display the graph
plt.show()
