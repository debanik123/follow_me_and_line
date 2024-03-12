# import numpy as np

# p = [9, 8, 7, 5, 3, 2, 1]

# def numbers_after_given_number(arr, given_number):
#     try:
#         index = arr.index(given_number)
#         return arr[index + 1:]
#     except ValueError:
#         print(f"The given number {given_number} is not present in the array.")
#         return []

# p = [9, 8, 7, 5, 3, 2, 1]
# given_number = 3

# result = numbers_after_given_number(p, given_number)
# print(result)

# p = [9, 8, 7, 5, 3, 2, 1]
# given_number = 5
# result = p[p.index(given_number) + 1:] if given_number in p else []

# print(result)

def numbers_after_given_number(arr, given_number):
    return (arr[arr.index(given_number):], arr.index(given_number)) if given_number in arr else ([], -1)

# Example usage:
p = [9, 8, 7, 5, 3, 2, 1]
print("p--> ", p)
given_number = 5

result, index = numbers_after_given_number(p, given_number)
print("Numbers after given number:", result)
print("Index of given number:", index)

# start_node = 16
# end_node = 14

# my_dict = {15: 'TuL', 13: 'TuR'}
# # print(**my_dict)
# # Insert at the beginning
# my_dict = {start_node: 'Start', **my_dict}

# # Insert at the end
# my_dict[end_node] = 'Goal'

# print(my_dict)

start_node, end_node = 16, 14
my_dict = {15: 'TuL', 13: 'TuR'}

my_dict = {start_node: 'Start', **my_dict, end_node: 'Goal'}

print(my_dict)