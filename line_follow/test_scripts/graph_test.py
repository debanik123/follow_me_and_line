import networkx as nx
import matplotlib.pyplot as plt

# Define the edges based on the connections you provided
edges = [
    (1,2), (2,3), (2,4), 
    (3,5), (4,7), (5,6), 
    (5,8), (7,6), (7,9), 
    (6,10), (10,11), (10,12), 
    (10,13)
]


# Create an undirected graph
G = nx.Graph()

# Add edges to the graph
G.add_edges_from(edges)

# Draw the graph
pos = nx.spring_layout(G)  # You can try different layout algorithms
nx.draw(G, pos, with_labels=True, node_size=700, node_color="skyblue", font_size=8, font_color="black", font_weight="bold", arrowsize=10)

# Display the graph
plt.show()