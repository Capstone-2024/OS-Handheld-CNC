
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(458, 213)
        Form.setMinimumSize(QtCore.QSize(458, 213))
        Form.setMaximumSize(QtCore.QSize(458, 213))
        # create the button setting button
        self.setting_btn = QtWidgets.QPushButton(Form)
        # button size
        self.setting_btn.setGeometry(QtCore.QRect(140, 20, 150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.setting_btn.setFont(font)
        self.setting_btn.setObjectName("setting_btn")
        # create the button information button
        self.information_btn = QtWidgets.QPushButton(Form)
        self.information_btn.setGeometry(QtCore.QRect(140, 80, 150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.information_btn.setFont(font)
        self.information_btn.setObjectName("information_btn")
        # create the button start button
        self.start_btn = QtWidgets.QPushButton(Form)
        self.start_btn.setGeometry(QtCore.QRect(140, 140, 150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.start_btn.setFont(font)
        self.start_btn.setObjectName("start_btn")

        self.retranslateUi(Form)
        # 连接信号和槽
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "CNC_UI"))
        self.setting_btn.setText(_translate("Form", "Settings"))
        self.information_btn.setText(_translate("Form", "Informations"))
        self.start_btn.setText(_translate("Form", "Start"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
