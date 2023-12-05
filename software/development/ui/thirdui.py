# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'thirdui.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(778, 457)
        Form.setStyleSheet("QLabel{\n"
"border:1px solid #cccccc;\n"
"\n"
"}")
        self.horizontalLayout = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSpacing(20)
        self.verticalLayout.setObjectName("verticalLayout")
        self.add_btn = QtWidgets.QPushButton(Form)
        self.add_btn.setMinimumSize(QtCore.QSize(150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.add_btn.setFont(font)
        self.add_btn.setObjectName("add_btn")
        self.verticalLayout.addWidget(self.add_btn)
        self.back_btn = QtWidgets.QPushButton(Form)
        self.back_btn.setMinimumSize(QtCore.QSize(150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.back_btn.setFont(font)
        self.back_btn.setObjectName("back_btn")
        self.verticalLayout.addWidget(self.back_btn)
        self.start_btn = QtWidgets.QPushButton(Form)
        self.start_btn.setMinimumSize(QtCore.QSize(150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.start_btn.setFont(font)
        self.start_btn.setObjectName("start_btn")
        self.verticalLayout.addWidget(self.start_btn)
        self.pushButton = QtWidgets.QPushButton(Form)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(Form)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_2.setMinimumSize(QtCore.QSize(150, 41))
        self.verticalLayout.addWidget(self.pushButton_2)
        spacerItem = QtWidgets.QSpacerItem(20, 128, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setSpacing(15)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.img_path_edit = QtWidgets.QLineEdit(Form)
        self.img_path_edit.setMinimumSize(QtCore.QSize(0, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.img_path_edit.setFont(font)
        self.img_path_edit.setObjectName("img_path_edit")
        self.verticalLayout_2.addWidget(self.img_path_edit)
        self.show_img_label = QtWidgets.QLabel(Form)
        self.show_img_label.setMinimumSize(QtCore.QSize(595, 377))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.show_img_label.setFont(font)
        self.show_img_label.setText("")
        self.show_img_label.setScaledContents(True)
        self.show_img_label.setAlignment(QtCore.Qt.AlignCenter)
        self.show_img_label.setObjectName("show_img_label")
        self.verticalLayout_2.addWidget(self.show_img_label)
        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "CNC_UI"))
        self.add_btn.setText(_translate("Form", "Add Svg File"))
        self.back_btn.setText(_translate("Form", "Back"))
        self.start_btn.setText(_translate("Form", "Start"))
        self.pushButton.setText(_translate("Form", "Scan"))
        self.pushButton_2.setText(_translate("Form", "Show Map"))
