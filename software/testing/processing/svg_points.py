from svgpathtools import svg2paths
import matplotlib.pyplot as plt

def svg_to_points_segment(svg_file, num_samples=5):
    # Convert SVG file to paths
    paths, attributes = svg2paths(svg_file)

    # Extract data points from paths
    x_coord = []
    y_coord = []
    count = 0 
    for path in paths:
        for segment in path:
            x_coord = [[]]*len(path)
            y_coord = [[]]*len(path)
            length = segment.length()
            num_segments = int(length)  # Number of line segments to approximate the line
            t_values = [i / num_segments for i in range(num_segments + 1)]
            for t in t_values:
                point = segment.point(t)
                x_coord[count].append(point.real)
                y_coord[count].append(point.imag)

    # Flip Y
    max_y = []
    for i in y_coord: 
        max_y.append(max(i))
    max_y = max(max_y)

    for i in range(len(y_coord)): 
        y_coord[i] = [(max_y - item) for item in y_coord[i]]

    print(x_coord, y_coord)
    
    scat = plt.scatter(x_coord, y_coord)
    plt.show()

    return x_coord, y_coord

def svg_to_points(svg_file, num_samples=5):
    # Convert SVG file to paths
    paths, attributes = svg2paths(svg_file)

    # Extract data points from paths
    x_coord = []
    y_coord = []

    for path in paths:
        for segment in path:
            length = segment.length()
            num_segments = int(length)  # Number of line segments to approximate the line
            t_values = [i / num_segments for i in range(num_segments + 1)]
            for t in t_values:
                point = segment.point(t)
                x_coord.append(point.real)
                y_coord.append(point.imag)

        # Flip Y
        y_coord = [(max(y_coord) - item) for item in y_coord]
    
    scat = plt.scatter(x_coord, y_coord)
    plt.show()

    return x_coord, y_coord

if __name__ == '__main__': 
    svg_to_points_segment("./software/testing/processing/test2.svg")