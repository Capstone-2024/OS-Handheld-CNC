import xml.etree.ElementTree as ET

def get_path_coordinates(svg_file_path):
    tree = ET.parse(svg_file_path)
    root = tree.getroot()

    # Find all path elements in the SVG file
    path_elements = root.findall(".//{http://www.w3.org/2000/svg}path")

    # Extract the coordinates from each path element
    for path in path_elements:
        if 'd' in path.attrib:
            path_data = path.attrib['d']
            path_coordinates = parse_path_coordinates(path_data)
            print(path_coordinates)

def parse_path_coordinates(path_data):
    # Remove unnecessary characters and split the path data
    path_data = path_data.replace('M', '').replace('L', ',').replace('Z', '')
    coordinates = path_data.split(',')

    # Convert coordinates to float values
    coordinates = [float(coord) for coord in coordinates]

    # Group coordinates into pairs
    pairs = [(coordinates[i], coordinates[i + 1]) for i in range(0, len(coordinates), 2)]

    return pairs

# Provide the path to your SVG file
svg_file_path = "Circle-01.svg"

# Call the function with the SVG file path
get_path_coordinates(svg_file_path)