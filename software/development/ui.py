import dearpygui.dearpygui as dpg
import cv2
import numpy as np
from ui_utils import scan_directory, file_selected_callback, updateConfig, readConfig

def to_rgb(bgr_img):
    return bgr_img[..., [2, 1, 0]]

def normalize(img):
    return img.astype(np.float32) / 255

# Create the main window
def main_page():

    # Variables
    window_width = 600
    window_height = 600

    dpg.create_context()

    with dpg.window(label="Main Page", pos=(0,0), width=window_width, height=window_height): 

        # Menu
        with dpg.menu_bar(): 
            with dpg.menu(label="Menu"):
                dpg.add_menu_item(label="About", callback=lambda:dpg.show_item("__about"))
            with dpg.menu(label="Start"):
                dpg.add_menu_item(label="Files", callback=lambda:dpg.show_item("__files"))
            with dpg.menu(label="Settings"):
                dpg.add_menu_item(label="Change Cut Settings", callback=lambda:dpg.show_item("__cut_settings"))
        

        # Picture View
        # could be changed
        video_file = 'data.mp4'
        cap = cv2.VideoCapture(video_file)
        assert cap.isOpened(), f'can not open video file {video_file}'
        _, first_frame = cap.read()
        h, w = first_frame.shape[:2]

        raw_data = np.zeros((h, w, 3), np.float32)
        with dpg.texture_registry():
            texture_id = dpg.add_raw_texture(w, h, raw_data, format=dpg.mvFormat_Float_rgb)
        with dpg.window(label="Main"):
            dpg.add_image(texture_id)

        # equal to dpg.start_dearpygui()
        if not dpg.is_viewport_created():
            dpg.setup_viewport()
        while (dpg.is_dearpygui_running()):
            print('loop----------')
            ret, bgr_img = cap.read()
            if not ret:
                break
            # 需要转换为rgb格式，并归一化到[0, 1]
            norm_rgb_img = normalize(to_rgb(bgr_img))
            raw_data[...] = norm_rgb_img[...]
            dpg.render_dearpygui_frame()

        dpg.cleanup_dearpygui()

        # Other Pages
        # About Us
        with dpg.window(label="About", show=False, tag="__about"): 
            dpg.add_text("Handheld CNC")
            pass

        # Files
        with dpg.window(label="Files", show=False, tag="__files",  pos=(0,0), width=window_width, height=window_height): 
            
            # Get the list of files in the directory
            directory = "./test_files"
            files = scan_directory(directory)

            for file in files: 
                dpg.add_button(label=str(file), callback=file_selected_callback, user_data=str(file), tag="__button_" + str(file))
                pass

        # Workpiece Scan
        
        
        # File Preview
        with dpg.plot(label="Cut Preview", show=False, height=window_height, width=window_width, tag="__cut_preview"):
            # REQUIRED: create x and y axes
            dpg.add_plot_axis(dpg.mvXAxis, label="x")
            dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="yaxis")

            dpg.add_scatter_series(0,0, label="Preview", parent="yaxis", tag="__cut_scatter")


        # Cut Settings

        # Read list of supported bits
        bit_list = []
        with open('supported_bits.txt') as f: 
            bit_list = [line.rstrip('\n') for line in f] 

        with dpg.window(label="Cut Settings", show=False, tag="__cut_settings", width=window_width, height=window_height): 
            # Bits Setting
            dpg.add_text("Supported Bits:")
            # List of Bits
            dpg.add_combo(items=bit_list, label="Bit List", callback=updateConfig, user_data=["settings.ini", "SETTINGS", "Bit"])

            # 

    dpg.create_viewport(title='CNC Control Panel', width=window_width, height=window_width)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()



if __name__ == "__main__": 
    main_page()