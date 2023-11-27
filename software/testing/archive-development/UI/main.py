

import sys

import qdarkstyle
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

from ui.firstui import Ui_Form as FirstUi
from ui.secondui import Ui_Form as SecondUi
from ui.thirdui import Ui_Form as ThirdUi
from ui.fourthui import Ui_Form as FourthUi
from ui.settingsui import Ui_Dialog as SettingsUi
from ui.informationui import Ui_Dialog as InformationUi
from settings import SettingsConfig

# BG_COLOR = '#7CA7AE'
SETTING_MANAGE = SettingsConfig()


class SettingsWin(QDialog, SettingsUi):

    def __init__(self, parent=None):
        super(SettingsWin, self).__init__(parent)
        self.setupUi(self)

        self.comboBox.currentTextChanged.connect(self.bits_change)

    def bits_change(self, bits_text):
        SETTING_MANAGE.set_param('BIT_TYPE', bits_text)
        QMessageBox.information(self, 'Success', 'Succeed to Set Bit Type')


class InformationWin(QDialog, InformationUi):
    def __init__(self, parent=None):
        super(InformationWin, self).__init__(parent)
        self.setupUi(self)


class FirstWin(QWidget, FirstUi):
    def __init__(self, parent=None):
        super(FirstWin, self).__init__(parent)
        self.setupUi(self)

        self.setting_btn.clicked.connect(self.pop_setting)
        self.information_btn.clicked.connect(self.pop_information)
        self.start_btn.clicked.connect(self.go_second)

    def go_second(self):
        self.second_win = SecondWin()
        self.second_win.show()
        self.close()

    def pop_setting(self):
        self.setting_win = SettingsWin()
        self.setting_win.show()

    def pop_information(self):
        self.information_win = InformationWin()
        self.information_win.show()


class SecondWin(QWidget, SecondUi):
    def __init__(self, parent=None):
        super(SecondWin, self).__init__(parent)
        self.setupUi(self)

        self.add_btn.clicked.connect(self.go_third)
        self.back_btn.clicked.connect(self.back_first)

    def back_first(self):
        self.first_win = FirstWin()
        self.first_win.show()
        self.close()

    def go_third(self):
        self.third_win = ThirdWin()
        self.third_win.show()
        self.close()


class ThirdWin(QWidget, ThirdUi):

    def __init__(self, parent=None):
        super(ThirdWin, self).__init__(parent)
        self.setupUi(self)

        self.img_path = None

        self.add_btn.clicked.connect(self.select_img)
        self.back_btn.clicked.connect(self.back_second)
        self.start_btn.clicked.connect(self.go_fourth)

    def back_second(self):
        self.second_win = SecondWin()
        self.second_win.show()
        self.close()

    def select_img(self):
        # 选择图片
        file_type_list = ['jpg', 'png', 'jpeg', 'PNG', 'JPG', 'JPEG']
        file_path, file_type = QFileDialog.getOpenFileName(self, 'Select Image', '.', 'All Files (*)')
        if not file_path:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        if file_path.split('.')[-1] not in file_type_list:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        self.img_path_edit.setText(file_path)
        self.show_img_label.setPixmap(QPixmap(file_path))
        self.img_path = file_path

    def go_fourth(self):
        if not self.img_path:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        self.fourth_win = FourthWin(img_path=self.img_path)
        self.fourth_win.show()
        self.close()


class FourthWin(QWidget, FourthUi):

    def __init__(self, parent=None, img_path=None):
        super(FourthWin, self).__init__(parent)
        self.setupUi(self)

        self.zoom_map = {
            'start_btn': True,
            'stop_btn': False,
        }

        self.zoom_frame.setEnabled(False)
        self.speed_label.setText(SETTING_MANAGE.get_param('SPEED'))
        self.bits_label.setText(SETTING_MANAGE.get_param('BIT_TYPE'))
        self.graphicsView.set_image(img_path)

        self.coordinates_edit.returnPressed.connect(self.zoom_to_coordinate)
        self.confirm_btn.clicked.connect(self.zoom_to_coordinate)
        self.reset_zoom_btn.clicked.connect(self.graphicsView.reset_zoom)
        self.start_btn.clicked.connect(self.enable_zoom)
        self.stop_btn.clicked.connect(self.enable_zoom)
        self.back_btn.clicked.connect(self.back_third)

    def enable_zoom(self):
        self.zoom_frame.setEnabled(self.zoom_map[self.sender().objectName()])

    def zoom_to_coordinate(self):
        text = self.coordinates_edit.text()
        if not text:
            QMessageBox.warning(self, 'Warning', 'Please Enter the Coordinates as (x, y)')
            return
        coordinates = text.split(",")
        if len(coordinates) == 2:
            x = int(coordinates[0].strip())
            y = int(coordinates[1].strip())
            self.graphicsView.zoom_to_coordinate(x, y)

    def back_third(self):
        self.third_win = ThirdWin()
        self.third_win.show()
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Fusion'))
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    win = FirstWin()
    win.show()
    sys.exit(app.exec_())
