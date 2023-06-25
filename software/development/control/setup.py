# Communicate with Arduino to send move commands 
from control.control_utils import ardu_write, ardu_read

def start_cut(file): 
    print("Cutting..." + file)

    # Show Stitched image/SVG?

    # 