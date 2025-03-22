from PyQt5.QtWidgets import QWidget, QPushButton, QLineEdit, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QPen
from pyqtgraph import PlotWidget, plot
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg
import random

class Dashboard(QWidget):
    def __init__(self, driver):
        super().__init__()
        self.init_ui()
        self.driver = driver
        self.data = []
        self.val = 0

    def init_ui(self):
        
        self.fq = 10
        # Set up the main layout
        self.main_layout = QVBoxLayout()

        # Create the canvas for the graph
        self.graphWidget = PlotWidget()
        self.graphWidget.setBackground('w')
        self.graphWidget.setTitle("Момент нагруски")
        self.graphWidget.setLabel('left', "М")
        self.graphWidget.setLabel('bottom', "t(ms)")

        layout = QVBoxLayout()
        layout.addWidget(self.graphWidget)

        self.main_layout.addLayout(layout)

        self.data_line = self.graphWidget.plot(pen=pg.mkPen(color=(255, 0, 0)))

        self.x = list(range(100))  # 100 data points
        self.y = [0] * 100
        self.data_line.setData(self.x, self.y)

        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start(self.fq)  # Update every 10ms

        self.graphWidget.setYRange(-10, 1100, padding=0)

        # Create the button and text field layout
        self.button_layout = QHBoxLayout()

        # Create the "Start" button
        self.start_button = QPushButton("Start")
        self.button_layout.addWidget(self.start_button)

        # Create the "Auto Rotate" button
        self.auto_rotate_button = QPushButton("Auto Rotate")
        self.button_layout.addWidget(self.auto_rotate_button)

        # Create the "Goto" button
        self.goto_button = QPushButton("Goto")
        self.button_layout.addWidget(self.goto_button)

        self.goto_field = QLineEdit()
        self.button_layout.addWidget(self.goto_field)

        # Create the text fields
        self.microstep_button = QPushButton("Microstep")
        self.button_layout.addWidget(self.microstep_button)
        self.microstep_field = QLineEdit()
        self.button_layout.addWidget(self.microstep_field)

        
        self.frequency_button = QPushButton("Frequency")
        self.button_layout.addWidget(self.frequency_button)
        self.frequency_field = QLineEdit()
        self.button_layout.addWidget(self.frequency_field)

        self.sgt_button = QPushButton("SGT")
        self.button_layout.addWidget(self.sgt_button)
        self.sgt_field = QLineEdit()
        self.button_layout.addWidget(self.sgt_field)

        self.current_field = QLineEdit()
        self.button_layout.addWidget(QLabel("Current"))
        self.button_layout.addWidget(self.current_field)

        # Add the button and text field layout to the main layout
        self.main_layout.addLayout(self.button_layout)

        # Set the main layout for the widget
        self.setLayout(self.main_layout)

        # Set the window properties
        self.setWindowTitle("Dashboard")
        self.resize(800, 600)

    def update_plot_data(self):
        self.x = self.x[1:]  # Remove the first x element.
        self.x.append((self.x[-1] + 1)%10000)  # Add a new value 1 higher than the last.

        if(self.data!=[]):
            val = str(self.data).split(",")  
            if(val[0]=="measure"):
                self.y = self.y[1:]  # Remove the first y element.
                self.val = (int)(val[1])
                self.y.append(self.val)  # Add a new random amplitude value.

        self.data_line.setData(self.x, self.y)

class Canvas(QWidget):
    def __init__(self, parent):
        super().__init__(parent)

        self.data = []

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 2))

        # Draw the graph
        for i in range(len(self.data) - 1):
            x1 = i * 5
            y1 = self.height() - (self.data[i] * 5)
            x2 = (i + 1) * 5
            y2 = self.height() - (self.data[i + 1] * 5)
            painter.drawLine(x1, y1, x2, y2)

    def update_data(self, new_data):
        self.data.append(new_data)
        self.update()