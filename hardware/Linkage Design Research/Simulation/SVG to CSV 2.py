import csv
from svg.path import parse_path

def convert_svg_to_csv(input_file, output_file):
    # Open the SVG file for reading
    with open(input_file, 'r') as svgfile:
        content = svgfile.read()

    # Parse the SVG path data
    path = parse_path(content)

    # Open the output CSV file for writing
    with open(output_file, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)

        # Write the header row
        csvwriter.writerow(['x', 'y'])

        # Extract the x and y coordinate points
        for point in path:
            x = point.start.real
            y = point.start.imag
            csvwriter.writerow([x, y])

# Convert SVG to CSV
convert_svg_to_csv('Circle-01.svg', 'output.csv')
