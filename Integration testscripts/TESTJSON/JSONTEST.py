import json # Used for reading and writing JSON. https://docs.python.org/3/library/json.html
import sys  # Used to read and write to console. https://docs.python.org/3/library/sys.html

# Read the input file name passed from the C# program via command line arguments
input_file_path = sys.argv[1]

# Read the input data from the file and deserialize the JSON string back into a 2D array of floats
with open(input_file_path, 'r') as input_file:
    input_data = json.load(input_file)

# Process the input data. Here we're just doubling each number for testing
output_data = [[x * 2 for x in row] for row in input_data]

# Serialize the output data into a JSON string and write it to the output file
output_file_path = 'output.json'
with open(output_file_path, 'w') as output_file:
    json.dump(output_data, output_file)

# Print the output file name to the console where it can be read by the C# program
print(output_file_path)
