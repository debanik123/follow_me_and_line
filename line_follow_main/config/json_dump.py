import json

edges = [
    (1, 2), (2, "i1"), (2, 3), (3, 4), (3, 5), (5, 6),
    (5, 7), (7, 8), (7, 10), (8, 9),
    (10, 13), (10, 11), (11, 12),
    (13, 14), (13, 15), (15, "i4"), (15, 16)
]

pos = {
    1: (0, -1),
    "i1": (-1, 0),
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
    14: (5, -1),
    15: (6, 0),
    "i4": (7, 0),
    16: (6, -1)
}

data = {
    "edges": edges,
    "pos": pos
}

# Convert to JSON
# json_data = json.dumps(data, indent=2)

# Print or save the JSON data
with open("factory2.json", "w") as json_file:
    json.dump(data, json_file, indent=2)

print("JSON data has been dumped to 'output.json'.")
