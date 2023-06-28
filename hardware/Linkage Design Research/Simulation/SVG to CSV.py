import csv
from svgpathtools import svg2paths, Line, Arc

def svg_to_csv(svg_file, csv_file, num_samples=100):
    # Convert SVG file to paths
    paths, attributes = svg2paths(svg_file)

    # Extract data points from paths
    data_points = []
    for path in paths:
        for segment in path:
            if isinstance(segment, Line):
                points = segment.sample(num_samples)
                for point in points:
                    data_points.append([point.real, point.imag])
            elif isinstance(segment, Arc):
                length = segment.length()
                num_segments = int(length)  # Number of line segments to approximate the arc
                t_values = [i / num_segments for i in range(num_segments + 1)]
                for t in t_values:
                    point = segment.point(t)
                    data_points.append([point.real, point.imag])

    # Write data points to CSV file
    with open(csv_file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y'])  # Write header
        writer.writerows(data_points)

    print(f"Conversion completed. Data points saved in '{csv_file}'.")

# Usage example
svg_file = 'Circle-01.svg'
csv_file = 'output.csv'
svg_to_csv(svg_file, csv_file)
