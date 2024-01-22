import networkx as nx
import matplotlib.pyplot as plt

def find_shortest_path(start, end, edges):
    G = nx.Graph()
    G.add_edges_from(edges)
    try:
        shortest_path = nx.shortest_path(G, source=start, target=end)
        return shortest_path
    except nx.NetworkXNoPath:
        return None

def draw_graph(edges, pos, highlight_path=None, start_node=None, end_node=None):
    G = nx.Graph()
    G.add_edges_from(edges)
    nx.draw(G, pos, with_labels=True, node_size=700, node_color="skyblue", font_size=8, font_color="black", font_weight="bold", arrowsize=10)
    
    if highlight_path:
        path_edges = list(zip(highlight_path, highlight_path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)
        
    if start_node:
        nx.draw_networkx_nodes(G, pos, nodelist=[start_node], node_color='green', node_size=700)
    if end_node:
        nx.draw_networkx_nodes(G, pos, nodelist=[end_node], node_color='orange', node_size=700)
        
    plt.show()

def junction_analysis(G, path):
    junction_nodes = []

    for i in range(1, len(path) - 1):
        current_node = path[i]
        neighbors = set(G.neighbors(current_node))
        
        if len(neighbors) > 2:
            junction_nodes.append(current_node)

    return junction_nodes

def get_relative_position(pos_dict, current_node, next_node):
    x_diff = pos_dict[next_node][0] - pos_dict[current_node][0]
    y_diff = pos_dict[next_node][1] - pos_dict[current_node][1]

    if x_diff >= 1 and y_diff ==0:
        return "forward"
    elif x_diff == 0 and y_diff >= 1:
        return "left"
    elif x_diff == 0 and y_diff <= -1:
        return "right"
    elif x_diff <= -1 and y_diff == 0:
        return "backward"
    else:
        return "unknown"

edges = [
    (1, 2), (2, 3), (2, 4),
    (3, 5), (4, 7), (5, 6),
    (5, 8), (7, 6), (7, 9),
    (6, 10), (10, 11), (10, 12),
    (10, 13)
]

pos = {
    1: (0, 0),
    2: (1, 0),
    3: (1, 1),
    4: (1, -1),
    8: (3, 2),
    5: (3, 1),
    6: (3, 0),
    7: (3, -1),
    9: (3, -2),
    11: (4, 1),
    10: (4, 0),
    12: (4, -1),
    13: (5, 0)
}

start_node = 1
end_node = 11

G = nx.Graph()
G.add_edges_from(edges)

path = find_shortest_path(start_node, end_node, edges)

if path:
    print(f"Shortest path from {start_node} to {end_node}: {path}")
    junction_nodes = junction_analysis(G, path)
    print("Junction nodes along the path:", junction_nodes)

    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        relative_position = get_relative_position(pos, current_node, next_node)
        print(f"From {current_node} to {next_node}: {relative_position}")

    draw_graph(edges, pos, highlight_path=path, start_node=start_node, end_node=end_node)

else:
    print(f"No path found from {start_node} to {end_node}")
