# Communicate with Arduino to send move commands 
from control.control_utils import ardu_write, ardu_read
from processing.svg import svg_to_points
import dearpygui.dearpygui as dpg

def start_cut(file): 
    print("Cutting..." + file)

    # Start scanning process for image stitching

    # feed file into SVG processor
    x, y = svg_to_points(file) # change file to absolute path, 100 samples default
    dpg.show_item('__cut_preview')
    dpg.set_value('__cut_scatter', [x, y])

    # transform points to real space, project onto stitched image


    # Show Stitched image/SVG?

    # 