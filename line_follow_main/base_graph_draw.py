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

def draw_graph(edges, pos, highlight_path=None, start_node=None, end_node=None, vertex_edge_dict=None):
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
    
        for vertex, info in vertex_edge_dict.items():
            plt.text(pos[vertex][0], pos[vertex][1], f"{vertex}\n({info})", ha='center', va='center', color='purple', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.3'))

        
    plt.show()

def junction_analysis(G, path):
    junction_nodes = []

    for i in range(1, len(path) - 1):
        current_node = path[i]
        neighbors = set(G.neighbors(current_node))
        
        if len(neighbors) > 2:
            junction_nodes.append(current_node)

    return junction_nodes

def get_direction(move):
    if move == "UU" or move == "DD" or move == "RR" or move == "LL":
        return "Fwd"  # Forward
    elif move == "UL" or move == "RU" or move == "DR" or move == "LD":
        return "TuL"  # Turning Left
    elif move == "UR" or move == "RD" or move == "DL" or move == "LU":
        return "TuR"  # Turning Right
    else:
        return "Unk"

def get_directions(path, pos):
    directions = []
    for i in range(len(path) - 1):
        current_node = path[i]
        next_node = path[i + 1]
        current_pos = pos[current_node]
        next_pos = pos[next_node]

        move = ""
        if current_pos[0] < next_pos[0]:
            move += "R"
        elif current_pos[0] > next_pos[0]:
            move += "L"
        if current_pos[1] < next_pos[1]:
            move += "U"
        elif current_pos[1] > next_pos[1]:
            move += "D"

        directions.append(move)

    return directions

def concatenate_adjacent_elements(lst):
    return [lst[i] + lst[i+1] for i in range(len(lst)-1)]

def create_vertex_edge_dict(path, moves, junction_nodes):
    vertex_edge_dict = {}

    for i in range(1, len(path) - 1):
        current_node = path[i]
        if current_node in junction_nodes:
            previous_node = path[i - 1]
            next_node = path[i + 1]
            edge_direction = moves[i - 1]
            
            vertex_edge_dict[current_node] = get_direction(moves[i - 1]+moves[i])   #edge_pn meaning the previous and next edge of the current node
            
    return vertex_edge_dict


edges = [(1,'r1'), ('r1', 2), (2,3), (2,4), 
(4, 'r2'), ('r2', 'r3'), ('r3', 5), 
(5, 'r4'), (5, 6), ('r4',7), 
(4, 8), (8,9) , (8, 10)]

pos = {
    1: (0, 0),
    'r1': (0,1),
    2: (1,1),
    3: (1,0),
    4: (2,1),
    'r2': (2,2),
    'r3': (3,2),
    5: (3,3),
    'r4': (2,3),
    6: (3,4),
    7: (2,4),
    8: (4,1),
    9: (4,0),
    10: (5,1)
}

start_node = 1
end_node = 9

print("Edges:", edges)
print("Pos:", pos)
print("Start Node:", start_node)
print("End Node:", end_node)

'''              
    (L,U) (U,L)  U   (U,R)  (R,U)
                 |
                 |
                 |
   L ____________|___________ R
                 |
                 |
                 |
                 |
     (L,D) (D,L) D  (D,R)  (R,D)

Up = U
Down = D
Left = L
Right = R

UU = DD = RR = LL = Forward
UL = RU = DR = LD = Turing Left
UR = RD = DL = LU =  Turing Right

'''

G = nx.Graph()
G.add_edges_from(edges)

path = find_shortest_path(start_node, end_node, edges)

if path:
    print(f"Shortest path from {start_node} to {end_node}: {path}")
    junction_nodes = junction_analysis(G, path)
    print("junction_nodes --> ",junction_nodes)

    moves = get_directions(path, pos)
    print(moves)

    concatenated_moves = concatenate_adjacent_elements(moves)
    print(concatenated_moves)

    if junction_nodes:
        vertex_edge_dict = create_vertex_edge_dict(path, moves, junction_nodes)
        print(vertex_edge_dict)
    
    draw_graph(edges, pos, highlight_path=path, start_node=start_node, end_node=end_node, vertex_edge_dict=vertex_edge_dict)


else:
    print(f"No path found from {start_node} to {end_node}")
