import sys
import cv2
from PyQt5.QtWidgets import *
from PyQt5 import uic
import matplotlib.pylab as plt

THRESH_ALL = 100


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = uic.loadUi("thresholding.ui", self)
        self.src_img = None             # 원본 영상
        self.res_img = None             # threshold 결과 영상
        self.src_title = "original"     # 원본 영상 제목
        self.res_title = "result image" # 결과 영상 제목
        self.rb_dict = {self.radioButton_binary: cv2.THRESH_BINARY,
                        self.radioButton_binary_inv: cv2.THRESH_BINARY_INV,
                        self.radioButton_trunc: cv2.THRESH_TRUNC,
                        self.radioButton_tozero: cv2.THRESH_TOZERO,
                        self.radioButton_tozero_inv: cv2.THRESH_TOZERO_INV,
                        }
        self.setup_ui()

    def setup_ui(self):
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave.triggered.connect(self.save_file)
        # set default values
        self.radioButton_binary.setChecked(True)
        self.verticalSlider.setMaximum(255)
        self.verticalSlider.setMinimum(0)
        self.verticalSlider.setValue(100)
        for rbutton in self.rb_dict.keys():
            rbutton.clicked.connect(self.threshold_image)
        self.verticalSlider.valueChanged.connect(self.threshold_image)
        self.checkBox_all_types.toggled.connect(self.show_all_types)

    def open_file(self):
        filename = QFileDialog.getOpenFileName(filter="JPG files (*.jpg)", directory="../sample_imgs")
        filename = filename[0]
        print("open file:", filename)
        if not filename:
            return
        self.src_img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        cv2.imshow(self.src_title, self.src_img)
        cv2.waitKey(1)
        self.threshold_image()

    def save_file(self):
        filename = QFileDialog.getSaveFileName(filter="JPG files (*.jpg)")
        filename = filename[0]
        print("save file:", filename)
        if not filename or self.res_img is None:
            return
        cv2.imwrite(filename, self.res_img)

    def threshold_image(self):
        if self.src_img is None:
            return
        thr_type, threshold = self.get_params()
        if thr_type == THRESH_ALL:
            self.res_img = self.threshold_all_types(threshold)
        else:
            ret, self.res_img = cv2.threshold(self.src_img, threshold, 255, thr_type)
        cv2.imshow(self.res_title, self.res_img)
        cv2.waitKey(1)

    def get_params(self):
        thr_type = cv2.THRESH_BINARY
        for rbutton, button_type in self.rb_dict.items():
            if rbutton.isChecked():
                thr_type = button_type
        threshold = self.verticalSlider.value()
        self.label_threshold.setText(f"Threshold: {threshold}")
        return thr_type, threshold

    def show_all_types(self, checked):
        if not checked:
            return
        threshold = self.verticalSlider.value()
        imgs = {"ORIGINAL": self.src_img}
        for rbutton, button_type in self.rb_dict.items():
            ret, res_binary = cv2.threshold(self.src_img, threshold, 255, button_type)
            imgs[rbutton.text()] = res_binary
        imgs['TRUNC'][0, 0] = 255

        for i, (key, value) in enumerate(imgs.items()):
            plt.subplot(2, 3, i+1)
            plt.title(key)
            plt.imshow(value, cmap='gray')
            plt.xticks([])
            plt.yticks([])
        plt.tight_layout()
        plt.show()


def main():
    app = QApplication(sys.argv)
    editor = MyWindow()
    editor.show()
    app.exec_()


if __name__ == "__main__":
    main()

