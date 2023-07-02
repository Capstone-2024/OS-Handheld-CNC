import csv
from svgpathtools import svg2paths
import os

def svg_to_points(svg_file, num_samples=100):
    # Convert SVG file to paths
    paths, attributes = svg2paths(os.getcwd() + "\\" + "test_files\\" + svg_file)

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
    y_max = max(y_coord)
    flip_y = []
    for i in range(0, len(x_coord)): 
        flip_y.append(y_max - y_coord[i])

    return x_coord, flip_y

    # Write data points to CSV file
    # with open(csv_file, 'w', newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(['x', 'y'])  # Write header

    #     for i in range(0, len(x_coord)): 
    #         writer.writerow([x_coord[i], y_max - y_coord[i]]) # flipped Y axis

    
    
    # print(f"Conversion completed. Data points saved in '{csv_file}'.")

# Usage example
if __name__ == "__main__": 
    svg_file = 'test2.svg'
    csv_file = 'output.csv'
    svg_to_points(svg_file) # , csv_file)
