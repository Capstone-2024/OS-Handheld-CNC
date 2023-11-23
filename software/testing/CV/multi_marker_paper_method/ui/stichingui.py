


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1051, 620)
        Form.setStyleSheet("QFrame#frame{\n"
"border:1px solid #cccccc;\n"
"}\n"
"QLabel{\n"
"border:1px solid #cccccc;\n"
"}")
        self.horizontalLayout = QtWidgets.QHBoxLayout(Form)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setSpacing(5)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame = QtWidgets.QFrame(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout.setContentsMargins(5, 5, 5, 5)
        self.verticalLayout.setSpacing(5)
        self.verticalLayout.setObjectName("verticalLayout")
        self.take_btn = QtWidgets.QPushButton(self.frame)
        self.take_btn.setMinimumSize(QtCore.QSize(121, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.take_btn.setFont(font)
        self.take_btn.setObjectName("take_btn")
        self.verticalLayout.addWidget(self.take_btn)
        self.save_btn = QtWidgets.QPushButton(self.frame)
        self.save_btn.setMinimumSize(QtCore.QSize(121, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.save_btn.setFont(font)
        self.save_btn.setObjectName("save_btn")
        self.verticalLayout.addWidget(self.save_btn)
        self.done_btn = QtWidgets.QPushButton(self.frame)
        self.done_btn.setMinimumSize(QtCore.QSize(121, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.done_btn.setFont(font)
        self.done_btn.setObjectName("done_btn")
        self.verticalLayout.addWidget(self.done_btn)
        self.verticalLayout_2.addWidget(self.frame)
        spacerItem = QtWidgets.QSpacerItem(20, 428, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.show_label = QtWidgets.QLabel(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.show_label.sizePolicy().hasHeightForWidth())
        self.show_label.setSizePolicy(sizePolicy)
        self.show_label.setMinimumSize(QtCore.QSize(881, 601))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.show_label.setFont(font)
        self.show_label.setText("")
        self.show_label.setScaledContents(True)
        self.show_label.setAlignment(QtCore.Qt.AlignCenter)
        self.show_label.setObjectName("show_label")
        self.horizontalLayout.addWidget(self.show_label)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "CNC_UI"))
        self.take_btn.setText(_translate("Form", "Take"))
        self.save_btn.setText(_translate("Form", "Save"))
        self.done_btn.setText(_translate("Form", "Done"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
