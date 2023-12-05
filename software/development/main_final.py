import sys
import time
import threading
import cv2
import os
import numpy as np
import pandas as pd
import math

# Import PyQT Functions
import qdarkstyle
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from ui.firstui import Ui_Form as FirstUi
from ui.secondui import Ui_Form as SecondUi
from ui.thirdui import Ui_Form as ThirdUi
from ui.fourthui import Ui_Form as FourthUi
from ui.stichingui import Ui_Form as StichingUi
from ui.settingsui import Ui_Dialog as SettingsUi
from ui.informationui import Ui_Dialog as InformationUi
from ui.mapui import Ui_Form as MapUi
from ui.warningui import Ui_Dialog as WarningUi
from ui.settings import SettingsConfig
from PyQt5 import QtCore, QtGui, QtWidgets
from threading_utils import WebcamVideoStream
from cv_utils import pose_estimation, plot_chart, manual_analyze_stitched, access_map
import numpy as np
from kalman_utils_3d import PE_filter
from sys import platform
from serial_utils import ArduinoComms

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import matplotlib
matplotlib.use('Agg')

# BG_COLOR = '#7CA7AE'
SETTING_MANAGE = SettingsConfig()


# define the settings
class SettingsWin(QDialog, SettingsUi):

    def __init__(self, parent=None):
        super(SettingsWin, self).__init__(parent)
        self.setupUi(self)
        
        # 建立连接
        # 在下拉框（comboBox）的文本变化时连接到 bits_change 
        self.comboBox.currentTextChanged.connect(self.bits_change)
    
    # 会调用 SETTING_MANAGE.set_param 方法, 用于将选择的比特类型设置为应用程序的设置，并弹出一个信息框表示成功设置。
    def bits_change(self, bits_text):
        SETTING_MANAGE.set_param('BIT_TYPE', bits_text)
        QMessageBox.information(self, 'Success', 'Succeed to Set Bit Type')


# define the information session
class InformationWin(QDialog, InformationUi):
    def __init__(self, parent=None):
        super(InformationWin, self).__init__(parent)
        # 调用 information的UI
        self.setupUi(self)


#define the warning session
class WarningWin(QDialog, WarningUi):

    def __init__(self, parent=None):
        super(WarningWin, self).__init__(parent)
        # 调用 Warning的UI
        self.setupUi(self)
        # self.style() 获取当前对话框的样式，并使用 standardIcon(QStyle.SP_MessageBoxWarning) 获取标准的消息框警告图标
        warning_icon = self.style().standardIcon(QStyle.SP_MessageBoxWarning)
        # 设置对话框的窗口图标为刚刚获取的警告图标
        self.setWindowIcon(warning_icon)
        # 设定图标的大小
        self.headicon_label.setPixmap(warning_icon.pixmap(48, 48))
        self.pushButton.clicked.connect(self.gonext)
        self.pushButton_2.clicked.connect(self.goback)
        
    # 关闭以后加载second UI
    def closeEvent(self, a0: QCloseEvent) -> None:
        # 创建 Second UI 对象
        self.second_win = SecondWin()
        # 显示 second UI
        # self.second_win.show()
        self.second_win.showFullScreen()
        # 关闭当前窗口
        self.close()

    def gonext(self, a0: QCloseEvent) -> None:
        # 创建 Second UI 对象
        self.second_win = SecondWin()
        # 显示 second UI
        # self.second_win.show()
        self.second_win.showFullScreen()
        # 关闭当前窗口
        self.close()
    
    def goback(self):
        # 创建 FirstWin
        self.first_win = FirstWin()
        # show the first UI
        # self.first_win.show()
        self.first_win.showFullScreen()
        # close UI
        self.close()


# stitching 以后 检查tool on workpiece or not?
# PromptWin(表示一个提示对话框)
class PromptWin(QMessageBox):

    def __init__(self, parent=None, title='CNC_UI', message='<p>Tool on WorkPiece ?</p>', icon=QMessageBox.Information):
        super(PromptWin, self).__init__(parent)
        # Two option
        self.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        font = QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        yes_btn = self.button(QMessageBox.Yes)
        no_btn = self.button(QMessageBox.No)
        self.setFont(font)
        yes_btn.setFont(font)
        no_btn.setFont(font)

        self.setWindowTitle(title)
        self.setText(message)
        self.setIcon(icon)

# first UI
class FirstWin(QWidget, FirstUi):
    def __init__(self, parent=None):
        super(FirstWin, self).__init__(parent)
        # 调用 first UI
        self.setupUi(self)

        # 连接setting_btn 到 pop_setting 
        self.setting_btn.clicked.connect(self.pop_setting)
        # 连接information_btn 到 pop_information
        self.information_btn.clicked.connect(self.pop_information)
        # 连接 start_btn 到 go_second
        self.start_btn.clicked.connect(self.go_second)
        self.pushButton.clicked.connect(self.quitApp)

    def go_second(self):
        # 创建 WarningWin
        self.warning_win = WarningWin()
        # 显示该窗口
        self.warning_win.show()
        # 关闭窗口
        self.close()

    def pop_setting(self):
        # 创建 SettingsWin
        self.setting_win = SettingsWin()
        # 显示该窗口
        self.setting_win.show()

    def pop_information(self):
        # 创建 InformationWin
        self.information_win = InformationWin()
        # 显示该窗口
        self.information_win.show()
    
    def quitApp(self):
        # 在槽函数中调用QApplication的quit方法来退出应用程序
        QApplication.quit()

# second UI
class SecondWin(QWidget, SecondUi):
    def __init__(self, parent=None):
        super(SecondWin, self).__init__(parent)
        self.setupUi(self)

        # add_btn 按钮的点击信号连接到 go_third
        self.add_btn.clicked.connect(self.go_third)
        # back_btn 按钮的点击信号连接到 back_first 
        self.back_btn.clicked.connect(self.back_first)

        self.pushButton.clicked.connect(self.point_tracking)
        self.pushButton_2.clicked.connect(self.smileface)

    def back_first(self):
        # 创建 FirstWin
        self.first_win = FirstWin()
        # show the first UI
        # self.first_win.show()
        self.first_win.showFullScreen()
        # close UI
        self.close()

    def go_third(self):
        # 创建 ThirdWin
        self.third_win = ThirdWin()
        #  show the third UI
        # self.third_win.show()
        self.third_win.showFullScreen()
        # close UI
        self.close()


    def point_tracking(self):
        
        self.close()
    
    def smileface(self):
        arduino_communicator = ArduinoComms()

        while arduino_communicator.homingOperation() != 'G': 
            # print(arduino_communicator.link.rxBuff)
            time.sleep(0.2)
            continue

        while arduino_communicator.zHoming() != 'O': 
            time.sleep(0.2)
            continue

        arduino_communicator.smiley()
        arduino_communicator.link.close()

# third UI
class ThirdWin(QWidget, ThirdUi):

    def __init__(self, parent=None):
        super(ThirdWin, self).__init__(parent)
        self.setupUi(self)

        # img_path 用于存储选择的图片的文件路径
        self.img_path = None

        # add_btn 按钮的点击信号连接到 select_img
        self.add_btn.clicked.connect(self.select_img)
        # back_btn 按钮的点击信号连接到 back_second 
        self.back_btn.clicked.connect(self.back_second)
        # start_btn 按钮的点击信号连接到 go_fourth
        self.start_btn.clicked.connect(self.go_fourth)
        self.pushButton.clicked.connect(self.scan_to)

    def back_second(self):
        # 创建 SecondWin
        self.second_win = SecondWin()
        #  show the second UI
        # self.second_win.show()
        self.second_win.showFullScreen()
        # close UI
        self.close()

    def select_img(self):
        # 允许选择的图片文件类型列表
        file_type_list = ['svg', 'SVG']
        # 打开文件对话框，获取选择的文件路径和文件类型
        file_path, file_type = QFileDialog.getOpenFileName(self, 'Select Image', '.', 'All Files (*)')
        # 如果用户取消选择，或者未选择文件，则弹出错误提示框
        if not file_path:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        # 检查文件类型是否在允许的列表中，如果不在则弹出错误提示框
        if file_path.split('.')[-1] not in file_type_list:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        # 在界面上显示选择的图片路径和图片
        self.img_path_edit.setText(file_path)
        self.show_img_label.setPixmap(QPixmap(file_path))
        # 将选择的图片路径存储在类的属性中，以便其他地方可以访问
        self.img_path = file_path

    def go_fourth(self):
        # 检查是否已选择了图片文件
        if not self.img_path:
            QMessageBox.critical(self, 'Error', 'Please Select an Image File!')
            return
        # 如果已选择图片文件，创建 FourthWin 的实例，并传递选择的图片路径作为参数
        self.fourth_win = FourthWin(img_path=self.img_path)
        # 显示第四个窗口，然后关闭当前的第三个窗口
        # self.fourth_win.show()
        self.fourth_win.showFullScreen()
        self.close()

    def scan_to(self):
        # 创建一个提示窗口
        prompt_win = PromptWin()
        # 显示提示窗口并等待用户响应
        result = prompt_win.exec_()
        # 创建一个贴合窗口
        self.sticking_win = StichingWin()
        # 根据用户响应采取相应的操作
        if result == QMessageBox.Yes:
            # 如果用户选择 Yes，则显示贴合窗口
            self.sticking_win.show()
        if result == QMessageBox.No:
            # 如果用户选择 No，则创建一个带有定制信息的提示窗口
            prompt_no_win = PromptWin(message='<p style="color:red;">Please place it down</p><p>Tool on WorkPiece ?</p>')
            # 显示新的提示窗口并等待用户响应
            dd_result = prompt_no_win.exec_()
            # 在用户选择 No 的情况下，持续显示新的提示窗口，直到用户选择了其他选项
            while dd_result == QMessageBox.No:
                dd_result = prompt_no_win.exec_()
            else:
                # 如果用户选择了其他选项，则显示贴合窗口
                self.sticking_win.show()


# fourth UI
class FourthWin(QWidget, FourthUi):

    def __init__(self, parent=None, img_path=None):
        super(FourthWin, self).__init__(parent)
        self.setupUi(self)
        
        # 用于存储按钮是否启用放大功能的状态
        self.zoom_map = {
            'start_btn': True,
            'stop_btn': False,
        }
        


        #摄像头
        self.cap_video=0
        #记录定时器工作状态
        self.flag = 0
        #存放每一帧读取的图像
        self.img = []
        
        # Vision thread
        self.video_thread = VideoThread()
        self.video_thread.frame_signal.connect(self.update_frame)

        self.timer = QTimer(self)
        self.start_btn.clicked.connect(self.execute_video_thread)
        self.stop_btn.clicked.connect(self.stop_video_thread)
        self.continue_btn.clicked.connect(self.continue_video)
        
        # 将 zoom_frame 控件设置为不可用状态
        self.zoom_frame.setEnabled(False)
        # 分别设置了两个标签 (speed_label 和 bits_label) 的文本内容，文本内容可能从某个管理对象 (SETTING_MANAGE) 中获取。
        self.speed_label.setText(SETTING_MANAGE.get_param('SPEED'))
        self.bits_label.setText(SETTING_MANAGE.get_param('BIT_TYPE'))
        #  调用 graphicsView 对象的 set_image 方法，可能用于设置图形视图的图像，其中参数 img_path 是从前一个窗口传递过来的图片路径
        # self.graphicsView.set_image(img_path)

        # 当用户在 coordinates_edit 文本框中输入完坐标并按下回车键时，会触发 returnPressed 信号，连接到 zoom_to_coordinate
        # self.coordinates_edit.returnPressed.connect(self.zoom_to_coordinate)
        # 当用户点击 scan_btn 按钮时，会触发 clicked 信号，连接到 scan_to
        self.scan_btn.clicked.connect(self.scan_to)
        # 当用户点击 confirm_btn 按钮时，会触发 clicked 信号，连接到 zoom_to_coordinate
        # self.confirm_btn.clicked.connect(self.zoom_to_coordinate)
        #  当用户点击 reset_zoom_btn 按钮时，会触发 clicked 信号，连接到 reset_zoom 
        # self.reset_zoom_btn.clicked.connect(self.graphicsView.reset_zoom)
        # 当用户点击 start_btn 或 stop_btn 按钮时，会触发 clicked 信号，连接到 enable_zoom
        self.start_btn.clicked.connect(self.enable_zoom)
        self.stop_btn.clicked.connect(self.enable_zoom)
        self.back_btn.clicked.connect(self.back_third)
        self.pushButton.clicked.connect(self.quitApp)
        self.pushButton_2.clicked.connect(self.accessMap)

    def scan_to(self):
        # 创建一个提示窗口
        prompt_win = PromptWin()
        # 显示提示窗口并等待用户响应
        result = prompt_win.exec_()
        # 创建一个贴合窗口
        self.sticking_win = StichingWin()
        # 根据用户响应采取相应的操作
        if result == QMessageBox.Yes:
            # 如果用户选择 Yes，则显示贴合窗口
            self.sticking_win.show()
        if result == QMessageBox.No:
            # 如果用户选择 No，则创建一个带有定制信息的提示窗口
            prompt_no_win = PromptWin(message='<p style="color:red;">Please place it down</p><p>Tool on WorkPiece ?</p>')
            # 显示新的提示窗口并等待用户响应
            dd_result = prompt_no_win.exec_()
            # 在用户选择 No 的情况下，持续显示新的提示窗口，直到用户选择了其他选项
            while dd_result == QMessageBox.No:
                dd_result = prompt_no_win.exec_()
            else:
                # 如果用户选择了其他选项，则显示贴合窗口
                self.sticking_win.show()

    # 根据按钮的状态设置 zoom_frame 控件的启用状态
    def enable_zoom(self):
        self.zoom_frame.setEnabled(self.zoom_map[self.sender().objectName()])

    def zoom_to_coordinate(self):
        # 获取文本框中的坐标信息
        text = self.coordinates_edit.text()
        
        # 如果文本框为空，弹出警告框并返回
        if not text:
            QMessageBox.warning(self, 'Warning', 'Please Enter the Coordinates as (x, y)')
            return
        
        # 解析坐标信息
        coordinates = text.split(",")
        if len(coordinates) == 2:
            x = int(coordinates[0].strip())
            y = int(coordinates[1].strip())
            # 调用图形视图的方法实现缩放到指定坐标
            self.graphicsView.zoom_to_coordinate(x, y)
    
    # 创建第三个窗口的实例并显示，然后关闭当前窗口
    def back_third(self):
        self.third_win = ThirdWin()
        self.third_win.show()
        self.close()
        
    def show_viedo(self):
        ret, self.img = self.cap_video.read()
        if ret:
            self.show_cv_img(self.img)
    
    def show_cv_img(self, img):
        pass

    def execute_video_thread(self):
        # Start VideoThread to execute once
        self.video_thread.start()
        self.timer.timeout.connect(self.update_video_frame)
        self.timer.start(50)
    
    def update_video_frame(self):
        # This method is called periodically to update the video frame
        if self.video_thread.isRunning():
            pass
        
    def update_frame(self, frame):
        height, width, channel = frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.label_3.setPixmap(pixmap)
        self.label_3.setScaledContents(True)  # Ensure the image scales to fit the label

    def closeEvent(self, event):
        self.timer.stop()
        self.video_thread.stop()
        self.video_thread.wait()
        event.accept()
    
    def stop_video(self):
        self.video_thread.pause()

    def continue_video(self):
        self.video_thread.resume()

    def stop_video_thread(self):
        # Stop the video thread
        self.video_thread.stop()
        self.timer.stop()

    def quitApp(self):
        # 在槽函数中调用QApplication的quit方法来退出应用程序
        QApplication.quit()

    def accessMap(self):
        data_dir = "your_data_directory_path"  # Replace with the actual data directory path
        map_data = access_map(data_dir)
        
        # Create an instance of MapWin and pass the map data
        self.map_win = MapWin(map_data)
        self.map_win.show()
        self.close()

        
        
class VideoThread(QThread):
    
    frame_signal = pyqtSignal(np.ndarray)
    
    def __init__(self):
        super(VideoThread, self).__init__()
        self.vs = cv2.VideoCapture(0)
        self.started = False  # 添加标志
        
        # Analyze Stitched Image, establishing global coordinate system
        self.marker_locations = manual_analyze_stitched()
        # data_dir = os.path.abspath("./data")
        # self.marker_locations = access_map(data_dir)

        ''' SET UP '''
        # self.point_i = 0 # index of target point 

        self.calibrate_settling_time = 5 # 5 Seconds for the filter to settle down
        self.calibrate_start_time = time.time()

        # Kalman Filter
        self.dt = 1 / 20  # 60 fps, i want to make this dynamic but idk if it works that way
        # P
        self.P_x = np.diag([1**2.0, 10**2, 10**2])  # covariance matrix
        self.P_y = np.diag([1**2.0, 10**2, 10**2])

        # R
        self.R_x = np.array([1**2, 20**2])
        self.R_y = np.array([1**2, 20**2])

        # Q
        self.Q = 30**2  # process variance

        self.x = np.array([0.0, 0.0, 0.0])
        self.kf_x = PE_filter(self.x, self.P_x, self.R_x, self.Q, self.dt)
        self.kf_y = PE_filter(self.x, self.P_y, self.R_y, self.Q, self.dt)

        # Initialize Communication with Arduino
        self.arduino = ArduinoComms()

        while self.arduino.homingOperation() != 'G': 
            time.sleep(0.3)
            continue

        while self.arduino.zHoming() != 'O': 
            time.sleep(0.3)
            continue

        num_accel_samples = 100
        offsets_x = np.zeros(num_accel_samples)
        offsets_y = np.zeros(num_accel_samples)
        
        # Accelerometer offsets
        for i in range(0, num_accel_samples): 
            _, x, y = self.arduino.regOperation()
            offsets_x[i] = x
            offsets_y[i] = y

        self.accel_offset_x = np.average(offsets_x)
        self.accel_offset_y = np.average(offsets_y)

        self.data_lock = QMutex()

    def run(self):
        # Main Loop
        self.started = True  # 设置标志

        while not self.isInterruptionRequested():
            ret, frame = self.vs.read()
            
            if ret:

                # Read low level status
                status, accel_x, accel_y = self.arduino.regOperation()

                # If button pressed
                if status == "Y": 
                    accel_x_mm = (accel_x - self.accel_offset_x)*1000
                    accel_y_mm = (accel_y - self.accel_offset_y)*1000
                    # accel_x_mm = 0 
                    # accel_y_mm = 0 

                    ''' Calculate Position with Pose Estimation '''
                    (x_pos, y_pos), z_rot, output = pose_estimation(frame, self.marker_locations)
                    
                    # Make sure all data are available
                    if (x_pos and y_pos and z_rot != None): 
                        # manual_offset = [marker_locations[17][0], marker_locations[17][1]] # Should only be in x or y, this is the position of the middle marker 
                        manual_offset = [50, 200]
                        # manual_offset = [200, 300] # Lower Resolution
                        x_pos = x_pos + manual_offset[0]
                        y_pos = y_pos + manual_offset[1]

                        # Kalman Filter Predict and Update
                        self.kf_x.predict()
                        self.kf_x.update([x_pos, accel_x_mm])
                        self.kf_y.predict()
                        self.kf_y.update([y_pos, accel_y_mm])
                        
                        # Calibrate for 5 seconds before running any tracking
                        if time.time() - self.calibrate_start_time >= self.calibrate_settling_time:

                            ''' Calculate Local Machine Error Vector and Send to Arduino '''
                            pos_diff = None

                            test_point = [0, 0]
                            pos_diff = [test_point[0] - self.kf_x.x[0], test_point[1] - self.kf_y.x[0]]

                            # print(f'Position Difference: {pos_diff[0], pos_diff[1]}')

                            ''' Send to Arduino '''
                            if abs(pos_diff[0]) <= 5 and abs(pos_diff[1]) <= 5: 
                                print('Sending to Arduino...')
                                print(f'Type: {pos_diff[0]}, Type: {pos_diff[0].item()}')
                                self.arduino.send_error(-pos_diff[0].item(), -pos_diff[1].item())
                                print(f'Data Sent: {pos_diff[0]}, {pos_diff[1]}')

                            ''' Testing '''
                            print(f'Current Global: {x_pos}, {y_pos}')
                            print(f'Filtered Global: {self.kf_x.x[0]}, {self.kf_y.x[0]} \n')
                            
                            ''' Send Frame Back to UI '''
                            small_frame = cv2.resize(output, (480, 270)) # Shrink Frame
                            convert = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
                            self.frame_signal.emit(convert)

        self.vs.release()
    
    def stop(self):
        if self.started:  # 检查标志
            self.vs.release()  # 释放摄像头资源
            super(VideoThread, self).stop()
    
    def pause(self):
        self.started = False

    def resume(self):
        self.started = True


# stitiching  UI
class StichingWin(QWidget, StichingUi):

    def __init__(self, parent=None):
        super(StichingWin, self).__init__(parent)
        self.setupUi(self)

        # 用于显示图片的变量
        self.show_pixmap = None

        # 连接按钮点击信号到槽函数
        self.take_btn.clicked.connect(self.capture_thread)
        self.save_btn.clicked.connect(self.save_photo)
        self.done_btn.clicked.connect(self.close)
        
        # 使用OpenCV库访问摄像头
        self.vs = cv2.VideoCapture(0)

        # 检查摄像头是否成功打开
        if not self.vs.isOpened():
            # 如果摄像头未成功打开，弹出错误提示框
            self.show_camera_error_message()
            return
        
        # Start a timer to capture frames periodically
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.capture)
        self.timer.start(50)

    def capture_thread(self):
        # Disable the "拍照" button to prevent multiple clicks
        self.take_btn.setEnabled(False)
        # Enable the "拍照" button after 3 seconds
        QTimer.singleShot(1000, lambda: self.take_btn.setEnabled(True))

    def capture(self):
        ret, frame = self.vs.read()

        # 如果成功读取到图像
        if ret:
            # 获取图像的高度、宽度和通道数
            height, width, channel = frame.shape
            # 计算每行的字节数
            bytes_per_line = 3 * width
            # 将OpenCV图像转换为Qt图像
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # convert to RGB from BGR
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            # 将Qt图像转换为QPixmap
            self.show_pixmap = QPixmap.fromImage(q_image)
            # 在界面上显示图片
            self.show_label.setPixmap(self.show_pixmap)

    def save_photo(self):
        # 检查是否有可保存的图片
        if self.show_pixmap is None:
            self.show_error_message('No Picture Can Be Saved!')
            return

        # 生成图片文件名（基于当前时间戳）
        img_name = str(int(time.time())) + '.png'
        try:
            # 保存图片到指定路径
            self.show_pixmap.save('./data/' + img_name)  # change name to image_i.png
        except Exception as e:
            # 如果保存出现异常，弹出错误提示框
            self.show_error_message(str(e))
        else:
            # 如果保存成功，弹出成功提示框
            QMessageBox.information(self, 'CNC_UI', 'Save Success!', QMessageBox.Ok)

    def show_error_message(self, message):
        t_box = QMessageBox(QMessageBox.Critical, 'Error', message, QMessageBox.Ok)
        font = QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        t_box.setFont(font)
        ok_btn = t_box.button(QMessageBox.Ok)
        ok_btn.setFont(font)
        t_box.exec_()

    def show_camera_error_message(self):
        self.show_error_message('USB Camera not Found!')

    def closeEvent(self, event):
        self.timer.stop()
        self.vs.release()
        event.accept()
        
class MapWin(QWidget, MapUi):
    def __init__(self, parent=None):
        super(MapWin, self).__init__(parent)
        # 调用 first UI
        self.setupUi(self)
        
        # Display the map data in the label
        self.display_map_data(map_data)


        # 连接setting_btn 到 pop_setting 
        self.map_back.clicked.connect(self.go_fourth)
    
    def display_map_data(self, map_data):
        map_label = self.map_label  

        # Clear any existing text in the label
        map_label.clear()

        # Convert the map_data dictionary to a string for display
        map_data_str = "\n".join([f"{key}: {value}" for key, value in map_data.items()])

        # Set the text in the QLabel to display the map data
        map_label.setText(map_data_str)


    def go_fourth(self):
        # 创建 WarningWin
        self.fourth_win = FourthWin()
        # 显示该窗口
        self.fourth_win.show()
        # 关闭窗口
        self.close()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle(QStyleFactory.create('Fusion'))
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    win = FirstWin()
    # win.show()
    win.showFullScreen()
    sys.exit(app.exec_())