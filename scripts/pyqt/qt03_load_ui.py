import sys
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QCoreApplication
from PyQt5 import uic


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # UI 불러오기
        self.ui = uic.loadUi("hellopyqt.ui", self)
        # signal - slot 연결
        self.btn_print.clicked.connect(self.hello_slot)
        self.btn_close.clicked.connect(QCoreApplication.instance().quit)
        self.count = 0
        print("window geometry:", self.geometry())
        print("btn_print position:", self.btn_print.pos())
        print("btn_print size:", self.btn_print.size())

    def hello_slot(self):
        self.count = self.count + 1
        self.label_print.setText(f"Hello PyQt {self.count}")


def main():
    app = QApplication(sys.argv)
    my_wnd = MyWindow()
    # my_wnd attribute 목록 출력하기
    print(dir(my_wnd))
    print([attr for attr in dir(my_wnd) if attr.startswith("btn") or attr.startswith("label")])
    my_wnd.show()
    app.exec_()


if __name__ == "__main__":
    main()
