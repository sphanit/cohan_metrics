#!/usr/bin/env python3
import PySimpleGUI as sg
from extract_metrics import MetricsData
import re

# Global variable to store the selected file path
file_path = None

# Regex pattern to find a float before the colon
pattern = r'([-+]?\d*\.\d+|\d+)\s*:' 
    
# Function to display the first line of the file
def display_log_start(file_path):
    try:
        with open(file_path, "r") as file:
            file.readline() # Skip the first line
            log_start = file.readline().strip()  # Read the first line
            log_start = re.search(pattern, log_start)
            start_time = log_start.group(1)
            window["-LOGSTART-"].update(start_time)  # Display first line
    except Exception as e:
        sg.popup(f"Error: {e}")

# Function to display the last line of the file
def display_log_end(file_path):
    try:
        with open(file_path, "r") as file:
            lines = file.readlines()  # Read all lines
            if lines:
                log_end = lines[-1].strip()  # Get the last line
                log_end = re.search(pattern, log_end)
                end_time = log_end.group(1)
                window["-LOGEND-"].update(end_time)  # Display last line
    except Exception as e:
        sg.popup(f"Error: {e}")

# Function to display the rest of the file content
def display_content(costs):
    try:
        window["-CONTENT-"].update("")  # Clear content field
        for hid, cost_array in costs.items():
            window["-CONTENT-"].print(hid)
            window["-CONTENT-"].print(cost_array)
    except Exception as e:
        sg.popup(f"Error: {e}")

# Layout of the GUI with additional options
layout = [
    [sg.Text("Select a file:"), sg.Input(size=(25, 1), key="-FILE-", enable_events=True), sg.FileBrowse()],
    [sg.Text("Log Start Time:"), sg.Text(size=(40, 1), key="-LOGSTART-")], 
    [sg.Text("Log End Time:"), sg.Text(size=(40, 1), key="-LOGEND-")],   
    [sg.Text("Human IDs:"), sg.Text(size=(40, 1), key="-HUMIDS-")],   
    [sg.Text("Options:")],
    [sg.Radio("Time Interval", "RADIO1", key="-TIMEINTERVAL-", enable_events=True, default=False)],
    [sg.Text("       Start Time:"), sg.Input(size=(10, 1), key="-STARTTIME-", disabled=True),
     sg.Text("       End Time:"), sg.Input(size=(10, 1), key="-ENDTIME-", disabled=True)],
    [sg.Radio("Tag Based", "RADIO1", key="-TAGBASED-", enable_events=True, default=True)],  # Default Tag Based
    [sg.Radio("Single", "RADIO2", key="-SINGLE-", disabled=False, default=True, enable_events=True), 
     sg.Text("Tag:"), sg.Input(size=(10, 1), key="-TAG-", disabled=False)],  # Enable Tag box by default
    [sg.Radio("Interval", "RADIO2", key="-INTERVAL-", disabled=False, enable_events=True), 
     sg.Text("Start Tag:"), sg.Input(size=(10, 1), key="-STARTTAG-", disabled=True), 
     sg.Text("End Tag:"), sg.Input(size=(10, 1), key="-ENDTAG-", disabled=True)],
    [sg.Button("Calculate"), sg.Button("Exit")],
    [sg.Multiline(size=(60, 10), key="-CONTENT-", no_scrollbar=True)],  # No scrollbar in multiline
]

# Create the window
window = sg.Window("CoHAN Metics", layout, finalize=True)

cohan_metrics = MetricsData()

# Event loop
while True:
    event, values = window.read()

    # If user closes window or clicks "Exit"
    if event == sg.WINDOW_CLOSED or event == "Exit":
        break
    
    # Accessing values from input boxes
    start_time = values["-STARTTIME-"]    # Access Start Time input
    end_time = values["-ENDTIME-"]        # Access End Time input
    tag = values["-TAG-"]                 # Access Tag input (for Single option)
    start_tag = values["-STARTTAG-"]      # Access Start Tag input (for Interval option)
    end_tag = values["-ENDTAG-"]          # Access End Tag input (for Interval option)

    # Handle radio button changes for Time Interval and Tag Based options
    if event == "-TIMEINTERVAL-":
        window["-STARTTIME-"].update(disabled=False)
        window["-ENDTIME-"].update(disabled=False)
        window["-SINGLE-"].update(disabled=True)
        window["-INTERVAL-"].update(disabled=True)
        window["-TAG-"].update(disabled=True)
        window["-STARTTAG-"].update(disabled=True)
        window["-ENDTAG-"].update(disabled=True)

    if event == "-TAGBASED-":
        window["-SINGLE-"].update(disabled=False)
        window["-INTERVAL-"].update(disabled=False)
        window["-STARTTIME-"].update(disabled=True)
        window["-ENDTIME-"].update(disabled=True)

        # Set "Single" as the default and clear other tag inputs
        window["-SINGLE-"].update(value=True)
        window["-TAG-"].update(value="", disabled=False)
        window["-STARTTAG-"].update(value="", disabled=True)
        window["-ENDTAG-"].update(value="", disabled=True)

    # Handle tag sub-option changes
    if event == "-SINGLE-":
        window["-TAG-"].update(disabled=False)
        window["-STARTTAG-"].update(disabled=True)
        window["-ENDTAG-"].update(disabled=True)

    if event == "-INTERVAL-":
        window["-TAG-"].update(disabled=True)
        window["-STARTTAG-"].update(disabled=False)
        window["-ENDTAG-"].update(disabled=False)

    # When a file is selected from FileBrowse
    if event == "-FILE-":
        file_path = values["-FILE-"]
        if file_path:
            display_log_start(file_path)  # Display the first line immediately
            display_log_end(file_path)   # Display the last line immediately
            cohan_metrics.load_data(file_path)
            window["-HUMIDS-"].update(cohan_metrics.get_human_ids())

    # When "Open" button is clicked, display the rest of the file content
    if event == "Calculate" and file_path:
        costs = cohan_metrics.calculate([1726778680.56172, 1726778700.25461])
        display_content(costs)

# Close the window
window.close()
