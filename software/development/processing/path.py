# Develop tool path based on input shape
# 1. Look at min and max xy and set start location
# 1.1 (0, 0) should be default
# 1.2 If starting is not (0, 0) for some reason, make svg 

# 2. Scale svg points input to available size on stitched image
# 2.1 Assume that 1 length in svg = stitched image size
# 2.2 Check if stitched image pixel (subtract marker pixel size, effectively a margin) is bigger than svg points
# 2.2 If so, proceed to next step
# 2.3 If not, then shrink the svg size down
# 2.3.1 Compare svg x and y lengths and compare stitched image x andy lengths, if they dont match, switch svg x and y . (If svg is square then dont do this)
# 2.3.2 Determine proportion between x and y on svg, then choose x or y to be set to the length of the stitched image (minus margin). multiply the other side by proportion

# 3. Perhaps project the svg onto the stitched image and show? could be useful 

# 4. Develop path (ignore)
# 4.1 Tell user to place machine at a specific location on the workpiece 
# 4.2 Position spindle to the right location via motor control (control should reach the desired start location) and drill 
# 4.3 Prompt user to start moving
# 4.4 To obtain next point, user moves 

# Develop path 
# - separating features (find closed paths)


