from graph_motion_planning import GraphAnalyzer

if __name__ == "__main__":
    # Create an object of the GraphAnalyzer class
    graph_analyzer = GraphAnalyzer('config/vtx_edg.json', 2,8)

    # Use the methods of the object
    path = graph_analyzer.find_shortest_path()

    if path:
        print(f"Shortest path from {graph_analyzer.start_node} to {graph_analyzer.end_node}: {path}")
        junction_nodes = graph_analyzer.junction_analysis(path)
        print("Junction nodes -->", junction_nodes)

        moves = graph_analyzer.get_directions(path)
        print("Moves:", moves)

        concatenated_moves = graph_analyzer.concatenate_adjacent_elements(moves)
        print("Concatenated Moves:", concatenated_moves)

        if junction_nodes:
            vertex_edge_dict = graph_analyzer.create_vertex_edge_dict(path, moves, junction_nodes)
            print("Vertex Edge Dict:", vertex_edge_dict)

            graph_analyzer.draw_graph(highlight_path=path, vertex_edge_dict=vertex_edge_dict)

    else:
        print(f"No path found from {graph_analyzer.start_node} to {graph_analyzer.end_node}")
