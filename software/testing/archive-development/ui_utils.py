import os
from control.setup import start_cut
import configparser

def scan_directory(directory):
    # Scans the specified directory for files
    files = []
    for file in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, file)):
            files.append(file)
    return files

def file_selected_callback(sender, app_data, user_data):
    # Starts Cut on the click item
    start_cut(user_data)


# Parse Configuration File

def readConfig(sender, app_data, user_data): 
    config = configparser.ConfigParser()

    file = user_data[0]
    section = user_data[1]
    key = user_data[2]

    # Read
    config.read(file)
    
    return config[section][key]

def updateConfig(sender, app_data, user_data): 
    config = configparser.ConfigParser()

    file = user_data[0]
    section = user_data[1]
    key = user_data[2]

    # File needs to exist first!
    config.read(file)
    print(config)

    # Update
    config[section][key] = app_data
    
    # Write to file
    with open(file, 'w') as configfile:
        config.write(configfile)
