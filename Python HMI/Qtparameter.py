import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QObject, pyqtSlot
import Dashboard

class Qtparameter(QObject):
    def __init__(self,  driver, parent=None,):
        super(Qtparameter, self).__init__(parent)
        self.driver = driver
        self.dashboard = parent
        self.start = False
        self.auto_rotate = False    

    @pyqtSlot()
    def on_start_button_clicked(self): 
        data_:str = "run "
        if(self.start):
            data_+="0"
            self.start = False
            self.dashboard.start_button.setText("Start")
        else:
            data_+="1"
            self.start = True
            self.dashboard.start_button.setText("Stop")
        self.driver.write_data(data_)

    @pyqtSlot()
    def on_auto_rotate_button_clicked(self):
        data_:str = "only_run "
        if(self.auto_rotate):
            data_+="0"
            self.auto_rotate = False
        else:
            data_+="1"
            self.auto_rotate = True
        self.driver.write_data(data_)

    @pyqtSlot()
    def on_goto_button_clicked(self):
        data_ = self.dashboard.goto_field.text()
        data_:str = "goto "+ data_
        self.driver.write_data(data_)

    @pyqtSlot()
    def on_microstep_field_editingFinished(self):
        pass

    @pyqtSlot()
    def on_microstep_button_clicked(self):
        data_ = str(self.dashboard.microstep_field.text())
        data_:str = "microstep "+ data_
        self.driver.write_data(data_)

    @pyqtSlot()
    def on_frequency_field_editingFinished(self):
        # Implement the logic for the Frequency field
        pass

    @pyqtSlot()
    def on_frequency_button_clicked(self):
        data_ = (str)(self.dashboard.frequency_field.text())
        self.dashboard.fq = int(data_)/1000
        data_:str = "frequency "+ data_
        self.driver.write_data(data_)

    @pyqtSlot()
    def on_current_field_editingFinished(self):
        data_ = (str)(self.dashboard.current_field.text())
        data_:str = "setCurrent "+ data_
        self.driver.write_data(data_)
    
    @pyqtSlot()
    def on_sgt_button_clicked(self):
        data_ = (str)(self.dashboard.sgt_field.text())
        data_:str = "sgt "+ data_
        self.driver.write_data(data_)