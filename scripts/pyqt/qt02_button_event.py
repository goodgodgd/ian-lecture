import sys
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QCoreApplication


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setupUI()

    def setupUI(self):
        self.setWindowTitle("PyQt: Basic button event")
        self.setGeometry(500, 200, 300, 200)
        print("window geometry:", self.geometry())
        print("window geometry:", self.geometry().x(), self.geometry().y(), self.geometry().width(), self.geometry().height())

        btn_print = QPushButton("Hello", self)
        btn_print.move(20, 20)
        btn_print.resize(100, 50)
        print("btn_print position:", btn_print.pos(), btn_print.pos().x(), btn_print.pos().y())
        print("btn_print size:", btn_print.size(), btn_print.size().width(), btn_print.size().height())
        btn_print.clicked.connect(self.hello_slot)

        btn_close = QPushButton("닫기", self)
        btn_close.move(20, 100)
        btn_close.clicked.connect(QCoreApplication.instance().quit)

    def hello_slot(self):
        print("Hello PyQt5")


def main():
    app = QApplication(sys.argv)
    mywindow = MyWindow()
    mywindow.show()
    app.exec_()


if __name__ == "__main__":
    main()
