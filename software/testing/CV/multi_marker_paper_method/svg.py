import csv
from svgpathtools import svg2paths
import os
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation
import numpy as np

def svg_to_points(svg_file, ratio, num_samples=5, view=False):
    print("Converting to points.")
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
    flip_y = [y - max(y_coord) for y in y_coord]

    # plot_x = [ x-min(x_coord) for x in x_coord]
    # plot_y = [ y-flip_y[x_coord.index(min(x_coord))] for y in flip_y]

    x_final = [i/ratio for i in x_coord]
    y_final = [i/ratio for i in y_coord]

    if view: 
        plt.scatter(x_final, y_final)
        plt.show()

    print("DONE")

    return x_final, y_final

    

    # Write data points to CSV file
    # with open(csv_file, 'w', newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(['x', 'y'])  # Write header

    #     for i in range(0, len(x_coord)): 
    #         writer.writerow([x_coord[i], y_max - y_coord[i]]) # flipped Y axis

    
    
    # print(f"Conversion completed. Data points saved in '{csv_file}'.")



# x_pts, y_pts = svg_to_points('test2.svg')
# fig, ax = plt.subplots()
# scat = ax.scatter(x_pts[0], y_pts[0])
# ax.set(xlim=[min(x_pts) - 50, max(x_pts)+ 50], ylim=[0 - 50, max(y_pts) + 50])

# def animation(frame): 
#     x = x_pts[:frame]
#     y = y_pts[:frame]

#     data = np.stack([x, y]).T
#     scat.set_offsets(data)

#     return scat

# ani = FuncAnimation(fig, animation, frames=len(x_pts), interval=1, repeat=False)
# plt.show()



# Usage example
# if __name__ == "__main__": 
#     svg_file = 'test2.svg'
#     csv_file = 'output.csv'
#     svg_to_points(svg_file) # , csv_file