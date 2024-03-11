import networkx as nx
import matplotlib.pyplot as plt
import json

class GraphAnalyzer:
    def __init__(self, file_path, start_node, end_node):
        self.file_path = file_path
        self.start_node = start_node
        self.end_node = end_node
        self.edges, self.pos = self.load_graph_data()

        self.G = nx.Graph()
        self.G.add_edges_from(self.edges)

        self.reverse = False

    def find_shortest_path(self):
        try:
            shortest_path = nx.shortest_path(self.G, source=self.start_node, target=self.end_node)
            return shortest_path
        except nx.NetworkXNoPath:
            return None

    def draw_graph(self, highlight_path=None, vertex_edge_dict=None):
        nx.draw(self.G, self.pos, with_labels=True, node_size=700, node_color="skyblue", font_size=8,
                font_color="black", font_weight="bold", arrowsize=10)

        if highlight_path:
            path_edges = list(zip(highlight_path, highlight_path[1:]))
            nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='red', width=2)

        if self.start_node:
            nx.draw_networkx_nodes(self.G, self.pos, nodelist=[self.start_node], node_color='green', node_size=700)
        if self.end_node:
            nx.draw_networkx_nodes(self.G, self.pos, nodelist=[self.end_node], node_color='orange', node_size=700)

        if vertex_edge_dict:
            for vertex, info in vertex_edge_dict.items():
                plt.text(self.pos[vertex][0], self.pos[vertex][1], f"{vertex}\n({info})", ha='center', va='center',
                         color='purple', bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.3'))

        plt.show()

    def junction_analysis(self, path):
        junction_nodes = []
        for i in range(1, len(path) - 1):
            current_node = path[i]
            neighbors = set(self.G.neighbors(current_node))
            if len(neighbors) > 2:
                junction_nodes.append(current_node)
        return junction_nodes

    def get_direction(self, move):
        
        if self.reverse:
            if move == "UU" or move == "DD" or move == "RR" or move == "LL":
                return "Fwd"  # Forward
            elif move == "UL" or move == "RU" or move == "DR" or move == "LD":
                return "TuR"  # Turning Left
            elif move == "UR" or move == "RD" or move == "DL" or move == "LU":
                return "TuL"  # Turning Right
            else:
                return "Unk"

        else:
            if move == "UU" or move == "DD" or move == "RR" or move == "LL":
                return "Fwd"  # Forward
            elif move == "UL" or move == "RU" or move == "DR" or move == "LD":
                return "TuL"  # Turning Left
            elif move == "UR" or move == "RD" or move == "DL" or move == "LU":
                return "TuR"  # Turning Right
            else:
                return "Unk"

    def get_directions(self, path):
        directions = []
        for i in range(len(path) - 1):
            current_node = path[i]
            next_node = path[i + 1]
            current_pos = self.pos[current_node]
            next_pos = self.pos[next_node]

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

    def concatenate_adjacent_elements(self, lst):
        return [lst[i] + lst[i + 1] for i in range(len(lst) - 1)]

    def create_vertex_edge_dict(self, path, moves, junction_nodes):
        vertex_edge_dict = {}
        for i in range(1, len(path) - 1):
            current_node = path[i]
            if current_node in junction_nodes:
                vertex_edge_dict[current_node] = self.get_direction(moves[i - 1] + moves[i])
        return vertex_edge_dict

    def load_graph_data(self):
        with open(self.file_path, 'r') as file:
            data = json.load(file)
            edges = [(edge[0], edge[1]) for edge in data['edges']]
            pos = {int(key): tuple(value) for key, value in data['poses'].items()}
            # start_node = data['start_node']
            # end_node = data['end_node']
        return edges, pos
    
