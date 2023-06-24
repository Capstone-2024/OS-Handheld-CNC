# Not sure what UI we want to do just yet
import dearpygui.dearpygui as dpg

# Render start page
# https://www.youtube.com/watch?v=-JZK8h-3bNk
def save_callback():
    print("Save Clicked")

dpg.create_context()
dpg.create_viewport()
dpg.setup_dearpygui()

with dpg.window(label="Example Window"):
    dpg.add_text("Click Start to Begin")
    dpg.add_button(label="Start", callback=save_callback)
    dpg.add_slider_float(label="float")

dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()