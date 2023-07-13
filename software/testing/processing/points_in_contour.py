import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import PatchCollection

def point_in_polygon(point, polygon):
    """
    Check if a point is inside a polygon.
    Implemented using the ray casting algorithm.
    """
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def distinguish_contours(points, cell_size):
    """
    Distinguish areas formed by contours in a set of points.
    """
    # Find boundaries
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)

    # Create grid
    cells = []
    for x in range(int(min_x), int(max_x) + 1, cell_size):
        for y in range(int(min_y), int(max_y) + 1, cell_size):
            cells.append([(x, y), (x + cell_size, y), (x + cell_size, y + cell_size), (x, y + cell_size)])

    # Check each cell against the contours
    labeled_cells = {}
    for cell in cells:
        for contour in contours:
            if point_in_polygon(cell[0], contour):
                labeled_cells[tuple(cell[0])] = True
                break
        else:
            labeled_cells[tuple(cell[0])] = False

    return labeled_cells

# Sample points (replace this with your own points)
points = [(1, 1), (1, 2), (2, 2), (2, 1), (3, 1), (3, 2), (4, 2), (4, 1), (2.5, 1.5)]
contours = [[(1, 1), (1, 2), (2, 2), (2, 1)], [(3, 1), (3, 2), (4, 2), (4, 1)]]
cell_size = 1  # Modify the cell_size to be an integer

# Distinguish contours
labeled_cells = distinguish_contours(points, cell_size)

# Prepare patches for visualization
patches = []
for cell, is_contour in labeled_cells.items():
    patches.append(patches.Rectangle(cell, cell_size, cell_size, fill=is_contour))

# Create plot
fig, ax = plt.subplots()
collection = PatchCollection(patches)
ax.add_collection(collection

# Set limits and aspect ratio
ax.set_aspect('equal')

# Show plot
plt.show()
