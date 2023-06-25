import dearpygui.dearpygui as dpg
from ui_utils import scan_directory, file_selected_callback, updateConfig, readConfig


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
        

        # Camera View?
        






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