import json

def dump_data_to_json(edges, poses, filename):
    data = {
        "edges": edges,
        "poses": poses
    }

    with open(filename, 'w') as json_file:
        json.dump(data, json_file, indent=2)


edges = [
        (1, 2), (2, "s1"), (2, 3), (3, 4), (3,5),(5, 6), 
        (5, 7), (7, 8), (7, 10), (8, 9), (8, "s5"),
        (10, 13), (10, 11), (11, 12), (11, "s6"),
        (13, 14), (13, 15), (15, "s4"), (15, 16)
    ]

poses = {
        1: (0, -1),
        "s1": (-1,0),
        2: (0, 0),
        3: (1, 0),
        4: (1, -1),
        5: (2, 0),
        6: (2, 2),
        7: (3, 0),
        8: (3, 2),
        9: (3, 3),
        10: (4, 0),
        11: (4, -1),
        12: (4, -2),
        13: (5, 0),
        14: (5, 1),
        15: (6, 0),
        "s4": (7, 0),
        16: (6, -1),
        "s5": (4,2),
        "s6":(3, -1)

    }

filename = "graph_data.json"
dump_data_to_json(edges, poses, filename)
