import sys
import re
from PyQt5.QtWidgets import (QApplication, QWidget, QLabel, QPushButton, 
                             QLineEdit, QFileDialog, QHBoxLayout, 
                             QTextEdit, QRadioButton, QButtonGroup, QGridLayout,
                             QSizePolicy)
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from extract_metrics import MetricsData

# Global variable to store the selected file path
file_path = None

# Regex pattern to find a float before the colon
pattern = r'([-+]?\d*\.\d+|\d+)\s*:'

class CoHANMetricsApp(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

        self.file_path = None
        self.cohan_metrics = MetricsData()

    def init_ui(self):
        self.setWindowTitle('CoHAN Metrics')

        # Main Layout
        grid_layout = QGridLayout()

        # File selection
        file_label = QLabel("Select a file:")
        self.file_input = QLineEdit(self)
        self.file_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred) 
        self.file_button = QPushButton('Browse', self)

        # Set the button's size policy to fixed
        self.file_button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
        self.file_button.clicked.connect(self.open_file)

        # Add file selection to grid layout
        grid_layout.addWidget(file_label, 0, 0)
        grid_layout.addWidget(self.file_input, 0, 1)  
        grid_layout.addWidget(self.file_button, 0, 2)

        # Log start and end display
        self.log_start_label = QLabel("Log Start Time:")
        self.log_start_value = QLabel("")
        self.log_end_label = QLabel("Log End Time:")
        self.log_end_value = QLabel("")

        grid_layout.addWidget(self.log_start_label, 1, 0)
        grid_layout.addWidget(self.log_start_value, 1, 1, 1, 2)
        grid_layout.addWidget(self.log_end_label, 2, 0)
        grid_layout.addWidget(self.log_end_value, 2, 1, 1, 2)

        # Options
        self.time_interval_radio = QRadioButton("Time Interval")
        self.tag_based_radio = QRadioButton("Tag Based", checked=True)

        self.time_interval_radio.toggled.connect(self.handle_radio_buttons)
        self.tag_based_radio.toggled.connect(self.handle_radio_buttons)

        self.single_radio = QRadioButton("Single", checked=True)  # Single checked by default
        self.interval_radio = QRadioButton("Interval")

        self.single_radio.toggled.connect(self.handle_tag_options)
        self.interval_radio.toggled.connect(self.handle_tag_options)

        tag_group = QButtonGroup(self)
        tag_group.addButton(self.single_radio)
        tag_group.addButton(self.interval_radio)

        # Inputs for different options
        self.start_time_input = QLineEdit(self)
        self.end_time_input = QLineEdit(self)
        self.tag_input = QLineEdit(self)
        self.start_tag_input = QLineEdit(self)
        self.end_tag_input = QLineEdit(self)

        # Time Interval Layout
        grid_layout.addWidget(self.time_interval_radio, 3, 0)

        grid_layout.addWidget(QLabel("Start Time:"), 3, 1)
        grid_layout.addWidget(self.start_time_input, 3, 2)
        grid_layout.addWidget(QLabel("End Time:"), 3, 3)
        grid_layout.addWidget(self.end_time_input, 3, 4)

        # Tag Based Layout
        grid_layout.addWidget(self.tag_based_radio, 4, 0)

        tag_layout = QHBoxLayout()
        tag_layout.addWidget(self.single_radio)
        tag_layout.addWidget(self.interval_radio)
        grid_layout.addLayout(tag_layout, 4, 1, 1, 3)

        # Single and Interval Layout
        grid_layout.addWidget(QLabel("Tag:"), 5, 1)
        grid_layout.addWidget(self.tag_input, 5, 2, 1, 2)

        grid_layout.addWidget(QLabel("Start Tag:"), 6, 1)
        grid_layout.addWidget(self.start_tag_input, 6, 2)
        grid_layout.addWidget(QLabel("End Tag:"), 6, 3)
        grid_layout.addWidget(self.end_tag_input, 6, 4)

        # Buttons
        self.calculate_button = QPushButton('Calculate', self)
        self.calculate_button.clicked.connect(self.get_metrics)
        self.exit_button = QPushButton('Exit', self)
        self.exit_button.clicked.connect(self.close)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.calculate_button)
        button_layout.addWidget(self.exit_button)
        grid_layout.addLayout(button_layout, 7, 0, 1, 5)

        # Text area to display content
        self.content_textbox = QTextEdit(self)
        self.content_textbox.setReadOnly(True)
        grid_layout.addWidget(self.content_textbox, 8, 0, 1, 5)

        self.setLayout(grid_layout)

        # Disable initially
        self.start_time_input.setEnabled(False)
        self.end_time_input.setEnabled(False)
        self.tag_input.setEnabled(True)
        self.start_tag_input.setEnabled(False)
        self.end_tag_input.setEnabled(False)

    def handle_radio_buttons(self):
        if self.time_interval_radio.isChecked():
            self.start_time_input.setEnabled(True)
            self.end_time_input.setEnabled(True)
            self.single_radio.setEnabled(False)
            self.interval_radio.setEnabled(False)
            self.tag_input.setEnabled(False)
            self.start_tag_input.setEnabled(False)
            self.end_tag_input.setEnabled(False)
        else:
            self.start_time_input.setEnabled(False)
            self.end_time_input.setEnabled(False)
            self.single_radio.setEnabled(True)
            self.interval_radio.setEnabled(True)
            if self.single_radio.isChecked():
                self.tag_input.setEnabled(True)
                self.start_tag_input.setEnabled(False)
                self.end_tag_input.setEnabled(False)
            else:
                self.tag_input.setEnabled(False)
                self.start_tag_input.setEnabled(True)
                self.end_tag_input.setEnabled(True)

    def handle_tag_options(self):
        if self.single_radio.isChecked():
            self.tag_input.setEnabled(True)
            self.start_tag_input.setEnabled(False)
            self.end_tag_input.setEnabled(False)
        elif self.interval_radio.isChecked():
            self.tag_input.setEnabled(False)
            self.start_tag_input.setEnabled(True)
            self.end_tag_input.setEnabled(True)

    def open_file(self):
        self.file_path, _ = QFileDialog.getOpenFileName(self, "Open File", "", "Text Files (*.txt);;All Files (*)")
        if self.file_path:
            self.file_input.setText(self.file_path)
            self.display_log_start(self.file_path)
            self.display_log_end(self.file_path)

    def display_log_start(self, file_path):
        try:
            with open(file_path, "r") as file:
                file.readline()  # Skip the first line
                log_start = file.readline().strip()  # Read the first line
                log_start = re.search(pattern, log_start)
                self.start_time = log_start.group(1)
                self.log_start_value.setText(self.start_time)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error: {e}")

    def display_log_end(self, file_path):
        try:
            with open(file_path, "r") as file:
                lines = file.readlines()  # Read all lines
                if lines:
                    log_end = lines[-1].strip()  # Get the last line
                    log_end = re.search(pattern, log_end)
                    self.end_time = log_end.group(1)
                    self.log_end_value.setText(self.end_time)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error: {e}")

    def get_metrics(self):
        if self.file_path:
            # Time Interval calculation logic
            if self.time_interval_radio.isChecked():
                start_time_int = self.start_time_input.text()
                end_time_int = self.end_time_input.text()
                self.cohan_metrics.load_data(self.file_path)
                if(not start_time_int and not end_time_int):
                    start_time_int = self.start_time
                    end_time_int = self.end_time
                costs = self.cohan_metrics.get_costs([start_time_int, end_time_int], type="time")
            
            # Tag-based calculation logic
            elif self.tag_based_radio.isChecked():
                if self.single_radio.isChecked():
                    tag = self.tag_input.text()
                    self.cohan_metrics.load_data(self.file_path, [tag, tag])
                    costs = self.cohan_metrics.get_costs([tag, tag], type="tag",)
                elif self.interval_radio.isChecked():
                    start_tag = self.start_tag_input.text()
                    end_tag = self.end_tag_input.text()
                    if self.file_path:
                        self.cohan_metrics.load_data(self.file_path, [start_tag, end_tag])
                        costs = self.cohan_metrics.get_costs([start_tag, end_tag], type="tag",)
                        self.display_content(costs)

            self.display_content(costs)
        else:
            QMessageBox.warning(self, "Error", "Please select a file first.")

    def display_content(self, cost_set):
        self.content_textbox.clear()
        try:
            for costs in cost_set:
                for hid, cost_array in costs.items():
                    self.content_textbox.append(f"{hid}\n{cost_array}\n")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = CoHANMetricsApp()
    window.show()
    sys.exit(app.exec_())
