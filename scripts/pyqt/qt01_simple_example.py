import sys
from PyQt5.QtWidgets import QApplication, QLabel
import time

app = QApplication(sys.argv)
label = QLabel("Hello PyQt")
label.show()
app.exec_()
