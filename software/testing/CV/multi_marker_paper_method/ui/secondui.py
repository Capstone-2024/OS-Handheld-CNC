

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(372, 181)
        Form.setMinimumSize(QtCore.QSize(372, 181))
        Form.setMaximumSize(QtCore.QSize(372, 181))
        self.add_btn = QtWidgets.QPushButton(Form)
        self.add_btn.setGeometry(QtCore.QRect(110, 30, 150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.add_btn.setFont(font)
        self.add_btn.setObjectName("add_btn")
        self.back_btn = QtWidgets.QPushButton(Form)
        self.back_btn.setGeometry(QtCore.QRect(110, 100, 150, 41))
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(12)
        font.setBold(True)
        font.setWeight(75)
        self.back_btn.setFont(font)
        self.back_btn.setObjectName("back_btn")

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "CNC_UI"))
        self.add_btn.setText(_translate("Form", "Choose"))
        self.back_btn.setText(_translate("Form", "Back"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())
