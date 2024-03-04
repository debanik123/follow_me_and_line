from graph_motion_planning import GraphAnalyzer
import pandas as pd

def create_node_job_dataframe(path, vertex_edge_dict, start_node, end_node):
    path_new = [node for node in path if isinstance(node, int)]
    df_data = {'Node': path_new, 'Job': ''}
    df = pd.DataFrame(df_data)
    for node, job in vertex_edge_dict.items():
        df.loc[df['Node'] == node, 'Job'] = job
    df.loc[df['Node'] == start_node, 'Job'] = 'Start'
    df.loc[df['Node'] == end_node, 'Job'] = 'Goal'
    
    return df

if __name__ == "__main__":
    # Create an object of the GraphAnalyzer class
    start_node = 1
    end_node = 4
    graph_analyzer = GraphAnalyzer('config/vtx_edg.json', start_node, end_node)

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
            if end_node not in junction_nodes:
                vertex_edge_dict = graph_analyzer.create_vertex_edge_dict(path, moves, junction_nodes)
                print("Vertex Edge Dict:", vertex_edge_dict)
                df = create_node_job_dataframe(path, vertex_edge_dict, start_node, end_node)
                print(df)
                graph_analyzer.draw_graph(highlight_path=path, vertex_edge_dict=vertex_edge_dict)
            else:
                print("000, Hi, I am executed")
                graph_analyzer.draw_graph(highlight_path=path, vertex_edge_dict=None)
        else:
            print("111, Hi, I am executed")
            graph_analyzer.draw_graph(highlight_path=path, vertex_edge_dict=None)

    else:
        print(f"No path found from {graph_analyzer.start_node} to {graph_analyzer.end_node}")
