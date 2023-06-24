import dearpygui.dearpygui as dpg
from ui_utils import scan_directory, file_selected_callback, updateConfig, readConfig


# Create the main window
dpg.create_context()

with dpg.window(label="Main Page", pos=(0,0), width=600, height=600): 

    # Menu
    with dpg.menu_bar(): 
        with dpg.menu(label="Menu"):
            dpg.add_menu_item(label="About", callback=lambda:dpg.show_item("__about"))
            
        with dpg.menu(label="File"):
            dpg.add_menu_item(label="Files", callback=lambda:dpg.show_item("__files"))
    
    # About Us
    with dpg.window(label="About", show=False, tag="__about"): 
        dpg.add_text("Handheld CNC")
        pass

    # Files
    with dpg.window(label="Files", show=False, tag="__files",  pos=(0,0), width=600, height=600): 
        
        # Get the list of files in the directory
        directory = "./test_files"
        files = scan_directory(directory)

        for file in files: 
            dpg.add_button(label=str(file), callback=file_selected_callback, user_data=str(file), tag="__button_" + str(file))
            pass
    # Settings
    with dpg.window(label="Bit Select"): 
        dpg.add_text("Supported Bits:")
        dpg.add_button(label="Pen", callback=updateConfig, user_data=["setting.ini", "SETTINGS", "Bit","pen"], tag="__button_pen")

dpg.create_viewport(title='CNC Control Panel', width=600, height=600)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()