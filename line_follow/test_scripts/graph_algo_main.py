import networkx as nx
import matplotlib.pyplot as plt

def find_shortest_path(start, end, edges):
    # Create an undirected graph
    G = nx.Graph()

    # Add edges to the graph
    G.add_edges_from(edges)

    try:
        # Find the shortest path
        shortest_path = nx.shortest_path(G, source=start, target=end)
        return shortest_path
    except nx.NetworkXNoPath:
        return None  # If there is no path between the nodes

def draw_graph(edges, highlight_path=None, start_node=None, end_node=None):
    # Create a graph
    G = nx.Graph()
    G.add_edges_from(edges)

    # Layout
    pos = {
    1: (0, 0),  # position of node 1
    2: (1, 0),  # position of node 2
    3: (2, 1),  # position of node 3
    4: (2, -1),  # position of node 4
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

    # Highlight the path, if specified
    if highlight_path:
        path_edges = list(zip(highlight_path, highlight_path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)

    # Mark start and end nodes
    if start_node:
        nx.draw_networkx_nodes(G, pos, nodelist=[start_node], node_color='green', node_size=700)
    if end_node:
        nx.draw_networkx_nodes(G, pos, nodelist=[end_node], node_color='orange', node_size=700)

    # Display the graph
    plt.show()

def junction_analysis(G, path):
    junction_nodes = []

    for i in range(1, len(path) - 1):
        current_node = path[i]
        neighbors = set(G.neighbors(current_node))

        # Check if the node is a junction
        if len(neighbors) > 1:
            junction_nodes.append(current_node)

    return junction_nodes
    
# Example usage
edges = [
    (1, 2), (2, 3), (2, 4),
    (3, 5), (4, 7), (5, 6),
    (5, 8), (7, 6), (7, 9),
    (6, 10), (10, 11), (10, 12),
    (10, 13)
]

start_node = 1
end_node = 13

G = nx.Graph()
G.add_edges_from(edges)

path = find_shortest_path(start_node, end_node, edges)

if path:
    print(f"Shortest path from {start_node} to {end_node}: {path}")
    junction_nodes = junction_analysis(G, path)
    print("Junction nodes along the path:", junction_nodes)
    draw_graph(edges, highlight_path=path, start_node=start_node, end_node=end_node)
    
else:
    print(f"No path found from {start_node} to {end_node}")
