import serial
from serial.tools import list_ports
import threading
from PyQt5.QtWidgets import QApplication
import sys
from Dashboard import Dashboard
from Qtparameter import Qtparameter

class Driver:
    def __init__(self, port, BAUD, OSCa_value=16000000): #16MHz
        self.arduino = serial.Serial(port, BAUD)
        self.arduino.close()        
        self.arduino = serial.Serial(port, BAUD)
        self.data_received = None
        self.reading_thread = threading.Thread(target=self.read_data)
        self.dashboard = None
    
    def read_data(self):
        while(True):
            if(self.arduino.in_waiting>0):
                self.data_received = self.arduino.readline().decode()
                self.dashboard.data = self.data_received
                #print(self.data_received)
                #self.data_received = self.data_received.split()
                #if(len(self.data_received)>1):
                #    self.serialTuple(self.data_received[0], self.data_received[1])
                #else:
                #    self.serialTuple(self.data_received[0], "")

    def write_data(self, data:str):
        self.arduino.write(data.encode())
    
def show_available_port():
    ports = list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))

def main():
    ## Configuration Port
    show_available_port()
    while(True):
        try:
            port_ = input("select port : ")
            driver = Driver(port_, 115200)
            break
        except:
            print('Port not available')

    app = QApplication(sys.argv)

    dashboard = Dashboard(driver)
    driver.dashboard = dashboard
    driver.reading_thread.start()
    parameter = Qtparameter(driver, dashboard)

    # Connect the signals and slots
    dashboard.start_button.clicked.connect(parameter.on_start_button_clicked)
    dashboard.auto_rotate_button.clicked.connect(parameter.on_auto_rotate_button_clicked)
    dashboard.goto_button.clicked.connect(parameter.on_goto_button_clicked)
    dashboard.microstep_button.clicked.connect(parameter.on_microstep_button_clicked)
    dashboard.microstep_field.editingFinished.connect(parameter.on_microstep_field_editingFinished)
    dashboard.frequency_button.clicked.connect(parameter.on_frequency_button_clicked)
    dashboard.frequency_field.editingFinished.connect(parameter.on_frequency_field_editingFinished)
    dashboard.current_field.editingFinished.connect(parameter.on_current_field_editingFinished)
    dashboard.sgt_button.clicked.connect(parameter.on_sgt_button_clicked)
    dashboard.show()

    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
