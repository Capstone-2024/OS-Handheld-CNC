from svgpathtools import svg2paths, Line, Arc, Path

def svg_to_points(svg_file, num_samples=100):
    # Convert SVG file to paths
    paths, attributes = svg2paths(svg_file)
    print(paths)

    # Extract data points from paths
    data_points = []
    for path in paths:
        for segment in path:
            if isinstance(segment, Path):
                points = segment.sample(num_samples)
                for point in points:
                    data_points.append([point.real, point.imag])
            elif isinstance(segment, Line):
                length = segment.length()
                num_segments = int(length)  # Number of line segments to approximate the arc
                t_values = [i / num_segments for i in range(num_segments + 1)]
                for t in t_values:
                    point = segment.point(t)
                    data_points.append([point.real, point.imag])

    # Calculate size of SVG by drawing a bounding box around the SVG paths
    print(data_points)

    return data_points
    # x, y

if __name__ == '__main__': 
    svg_to_points("test.svg")