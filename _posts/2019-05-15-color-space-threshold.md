---
layout: post
title:  "[Python] Color Space and Thresholding"
date:   2019-05-15 09:00:01
categories: 2019-1-systprog
---



# 1. Color Space

Color space란 화소의 색상을 표현하는 공간이다. (영상 픽셀 좌표 (u, v)와는 전혀 다른 개념이다.) 우리가 가장 흔하게 쓰고 있는 color space는 RGB color space다. 일반적인 영상에서 한 픽셀은 RGB가 각각 8 bit씩 0~255 사이의 값을 가질 수 있다. 아래 그림처럼 하나의 색상은 3차원 RGB 공간에서 한 점을 나타낸다. 

![rgb-color-space](/ian-lecture/assets/opencv-color/rgb-color-space.gif)



## 1.1 Binary, Gray

이렇게 색상을 나타낼 수 있는 공간을 color space라고 하는데 그 중 가장 간단한 것은 흑과 백, 즉 0과 255로만 이루어진 **binary** 영상이다. 흰 바탕에 검은 글씨만 쓸 수 있는 메모장 같은 앱이 binary color space를 쓴다고 볼 수 있다. 그리고 밝기의 명암을 나타낼 수 있는 **gray** 영상이있다. Binary와 gray 모두 픽셀 하나의 색상이 하나의 값으로 결정이 되는 1차원 color space다. binary는 값을 0과 255만 가질수 있고 gray는 0~255 사이의 값을 가질 수 있다. 아래 그림은 binary, gray, rgb 영상을 비교한 것이다.

![spiderman-gray](/ian-lecture/assets/opencv-color/spiderman-gray.jpg)



컬러 영상을 표현하기 위해서는 화소 하나당 RGB 세 개의 값이 필요하다. 즉 red, green, blue 영상을 나타내는 세 장의 gray 영상이 합쳐져야 하나의 컬러 영상을 만들 수 있다. 이때 각 색상을 나타내는 gray 영상을 **채널(channel)**이라 부른다.  영상 데이터의 shape을 보면 gray 영상은 (rows, cols) 2차원 배열로 나타나지만 rgb 영상은 (rows, cols, 3)으로 3차원 배열로 나타난다. 아래 그림은 왼쪽부터 blue, green, red 각 채널을 gray 영상으로 나타낸 것이다. Blue와 red 영상을 보면 스파이더맨 슈트의 빨간색과 파란색 영역의 밝기가 서로 반대로 나타나는 것을 확인할 수 있다.

![spiderman-bgr](/ian-lecture/assets/opencv-color/spiderman-bgr.jpg) 



위 영상을 만드는 코드는 다음과 같다. Color space 변환에는 `cvtColor`라는 함수를 이용한다.

> **cvtColor(src, code)**:  `src`는 변환할 영상이고 `code`는 입력과 출력 color space를 나타낸다. `cv2.COLOR_[입력 color]2[출력 color]` 형식으로 정의되어 있다. 예를 들어 BGR에서 Gray로 변환하는 코드는 `cv2.COLOR_BGR2GRAY`다. OpenCV는 수백가지의 color space 변환을 지원하는데 전체 목록은 [이곳](<https://docs.opencv.org/4.0.0/d8/d01/group__imgproc__color__conversions.html>)에서 확인할 수 있다.

```python
import os
import cv2
import numpy as np

IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "spiderman.jpg")
bgrimg = cv2.imread(filename)
# color space 변환
grayimg = cv2.cvtColor(bgrimg, cv2.COLOR_BGR2GRAY)
# binary image 만들기
binaryimg = grayimg.copy()
binaryimg[grayimg < 120] = 0
binaryimg[grayimg >= 120] = 255
# 3채널 bgrimg 와 합치기 위해 다시 BGR로 변환
grayimg = cv2.cvtColor(grayimg, cv2.COLOR_GRAY2BGR)
binaryimg = cv2.cvtColor(binaryimg, cv2.COLOR_GRAY2BGR)
# axis=1 가로로 합치기
concatimg = np.concatenate([binaryimg, grayimg, bgrimg], axis=1)
cv2.imshow("binary gray BGR", concatimg)
cv2.waitKey()
# BGR 채널별로 나누고 다시 가로로 합치기
channels = np.concatenate([bgrimg[:,:,0], bgrimg[:,:,1], bgrimg[:,:,2]], axis=1)
cv2.imshow("BGR channels", channels)
cv2.waitKey()
```



그런데 색상을 표현할 수 있는 color space가 RGB만 있는 것은 아니다. 3차원 공간을 나타내는 좌표계가 cartesian coordinate만 있는게 아니라 poloar coordinate, spherical coordinate 등도 있듯이 color space의 종류도 여러가지가 있다. 이제부터 하나씩 알아보자. 여기서 다양한 color space를 배우는 이유는 **color space의 종류마다 특성과 쓰임이 다르므로 상황에 따라 적절한 color space를 변환한 후 영상처리를 하는 것이 유리**하기 때문이다.



## 1.2 HSV, HSI

**HSV** color space는 색상을 Hue(색조), Saturation(채도), Value(명도)로 나타낸 것으로 Value 대신 Intensity를 써서 HSI라 부르기도 한다. 각 채널의 특성은 다음과 같다.

- **H**ue: 픽셀의 색을 표현. 0~180 사이의 값을 가지며 0과 180 근처가 빨간색, 60 내외가 초록색, 105 주변이 파란색을 나타낸다.
- **S**aturation: 색의 순도. 0~255 사이의 값을 가지며 0에 가까울수록 무채색이되고 255에 가까울수록 색이 선명해진다.
- **V**alue: 빛의 밝기. 0~255사이의 값을 가지며 0에 가까울수록 어둡고 255에 가까울수록 밝다.

![HSV-color-space](/ian-lecture/assets/opencv-color/HSV_color_space.png)



이번에도 `cvtColor()` 함수를 써서 color space를 변환하였다. BGR에서 HSV로 변환하는 코드는 `cv2.COLOR_BGR2HSV` 이다.

```python
# HSV color space 변환
hsvimg = cv2.cvtColor(bgrimg, cv2.COLOR_BGR2HSV)
# HSV 채널별로 나누고 다시 가로로 합치기
channels = np.concatenate([hsvimg[:,:,0], hsvimg[:,:,1], hsvimg[:,:,2]], axis=1)
cv2.imshow("HSV channels", channels)
cv2.waitKey()
filename = os.path.join(IMG_PATH, "spiderman-hsv.jpg")
cv2.imwrite(filename, channels)
```

![spiderman-hsv](/ian-lecture/assets/opencv-color/spiderman-hsv.jpg)

각 채널 영상을 원본 영상과 비교해보면 이해할 수 있다. 왼쪽은 Hue 채널인데 빨간색이 0이나 180 근처이므로 옷의 빨간색은 180에 가깝게 나타났고 얼굴은 0에 가깝게 나왔다. 그리고 옷 영역에서 명암에 따른 차이는 거의 사라지고 색상별로 거의 균일한 값이 나옴을 확인할 수 있다. 오른쪽은 Value 채널인데 밝기를 나타낸 것이므로 gray 영상과 비슷하게 나온다.  

이걸로 무엇을 할 수 있을까? 영상 영역을 같은 의미 단위로 나누는 것을 segmentation 이라고 한다. HSV color space에서는 Hue 채널을 이용하여 특정 색상을 가진 영역을 추출할 수 있다. Hue 영상을 보면 얼굴이 있는 부분은 hue 값이 낮고 가슴의 빨간 색 영역은 hue값이 높은 것을 확인할 수 있다. 이를 이용하여 스파이더맨을 부위별로 segmentation이 가능하다.  

```python
hueimg = hsvimg[:,:,0]
valueimg = hsvimg[:,:,2]
faceimg = np.zeros(bgrimg.shape, dtype=np.uint8)
# hue 값이 낮은 얼굴 영역 추출
faceimg[hueimg<13, :] = bgrimg[hueimg<13, :]
bodyimg = np.zeros(bgrimg.shape, dtype=np.uint8)
# hue 값이 높은 슈트의 빨간 영역 추출
bodyimg[hueimg>170, :] = bgrimg[hueimg>170, :]
# hue 값이 100~130인 슈트의 파란 영역 추출
bodyimg[(hueimg>100) & (hueimg<130), :] = bgrimg[(hueimg>100) & (hueimg<130), :]
bodyimg[valueimg>220] = 0
segment = np.concatenate([faceimg, bodyimg], axis=1)
cv2.imshow("segmented image", segment)
cv2.waitKey()
```

![spiderman-segment](/ian-lecture/assets/opencv-color/spiderman-segment.jpg)

왼쪽의 톰 홀랜드는 색상이 선하고 배경이 단순하기 때문에 거의 정확하게 얼굴과 몸이 구분된 것을 볼 수 있다. 반면 오른쪽의 토비 맥과이어는 색상이 어두워서 색상의 해상도가 좋지 않고 배경에도 파란색이 들어가 있어서 segmentation의 정확도가 떨어진다. 여기서 중요한 것은 **양쪽의 명암이 크게 다른데도 색상(hue)을 기준으로 비슷한 결과**를 냈다는 것이다.  

이러한 segmentation을 BGR에서도 구현할 수 있을까? BGR에서 빨간 영역을 추출하려면 R이 높고 BG는 낮은 영역을 찾아야 할 것 이다. 하지만 R값 자체도 명암에 따라 크게 달라지고 토비 맥과이어처럼 어두운 영역까지 추출하려고 한다면 범위가 너무 커져서 다른 영역까지 함께 추출될 것이다. 따라서 BGR은 화소 값 기반 segmentation에  적합한 color space가 아니다.  



## 1.3 YUV, YCbCr

YUV는 사람이 색상을 인식 할 때 색상보다는 밝기에 더 민감하다는 점을 고려해서 만든 color space다. 채널 구성은 다음과 같다.

- **Y** (Luma): 빛의 밝기, OpenCV에서는 8bit를 할당하지만 원래는 이곳에 더 많은 bit를 할당하여 밝기 해상도를 높인다.
- **U** (Chroma Blue, Cb): 밝기와 파란색과의 색상 차
- **V** (Chroma Red, Cr): 밝기와 빨간색과의 색상 차

YUV는 TV 방송용 아날로그 컬러 인코딩에서 유래됐고 YCbCr은 MPEG이나 JPEG 같은 디지털 컬러를 인코딩하는데 사용되었으나 방식이 비슷해서 오늘날에는 혼용되고 있다. OpenCV 컬러 변환 코드도 `cv2.COLOR_BGR2YUV`와 `cv2.COLOR_BGR2YCbCr` 두 가지가 있고 결과도 미세하게 다르다. YUV는 대표적으로 명암 대비(contrast)를 개선하는데 활용되는데 여기서는 변환 결과만 보도록 한다. ~~지금 새벽 3시니까~~

```python
# YUV color space 변환
yuvimg = cv2.cvtColor(bgrimg, cv2.COLOR_BGR2YUV)
# YUV 채널별로 나누고 다시 가로로 합치기
channels = np.concatenate([yuvimg[:,:,0], yuvimg[:,:,1], yuvimg[:,:,2]], axis=1)
cv2.imshow("YUV channels", channels)
cv2.waitKey()
```

![spiderman-yuv](/ian-lecture/assets/opencv-color/spiderman-yuv.jpg)



# 2. Thresholding

Thresholding 이란 특정 값을 기준으로 binary 영상을 만드는 기법이다. 영상에서 중간값을 없애고 흑백의 극단으로만 표현하면 사물의 윤곽을 뚜렷하게 나타낼 수 있다. 혹은 글자나 표지판처럼 원래 binary 영상이었으나 사진을 찍으면서 흐릿해진 부분을 다시 선명하게 복원할 때도 사용된다.  

앞서 처음 보여준 예시처럼 numpy 인덱싱을 통해서도 thresholding을 구현할 수 있지만 OpenCV에서 다양한 thresholding 방법을 편리하게 쓸 수 있게 만들어준 `threshold()`라는 함수를 제공한다. 결과를 보면 영상이 흑백으로 단순화된 것을 볼 수 있다. 

```python
import os
import cv2
import numpy as np

IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "sungmo.jpg")
image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
ret, binary = cv2.threshold(image, 180, 255, cv2.THRESH_BINARY)
showimg = np.concatenate([image, binary], axis=1)
cv2.imshow("thresolding", showimg)
cv2.waitKey()
```

![sungmo_thresh](/ian-lecture/assets/opencv-color/sungmo_thresh.png)

`threshold` 함수의 입출력 인자는 다음과 같다.

> ret, dst cv2.threshold(src, thresh, maxval, type)  
>
> - img: 입력 영상
> - threshold: 경계값
> - value: 경계 값 기준을 만족하는 픽셀에 들어갈 값
> - type: threshol 적용 방법
>   - cv2.THRESH_BINARY: px > threshold ? maxval: 0
>   - cv2.THRESH_BINARY_INV: px > threshold ? 0 : maxval
>   - cv2.THRESH_TRUNC: px > threshold ? maxval : px
>   - cv2.THRESH_TOZERO: px > threshold ? px : 0
>   - cv2.THRESH_TOZERO_INV: px > threshold ? 0 : px
> - ret: `thresh` 값 출력
> - dst: thresholding 결과 영상



## 2.1 기본 GUI 만들기

`threshold()` 함수의 다양한 효과를 편리하게 확인하기 위해 GUI를 만들어보자. `QFileDialog`를 통해 파일을 선택하기 위해 `menubar`에 `File-Open, Save`를 추가하고, `thresh`는 slider를 통해 조절할 수 있다. `maxval`은 255 고정값으로 넣고, `type`은 여러가지 중에 하나를 선택해야 하니 radio button을 사용한다. 그래서 다음과 같은 GUI를 QtDesigner에서 설계한 뒤 `thresholding.ui`라는 이름으로 저장한다.

![threshold-gui](/ian-lecture/assets/opencv-color/threshold-gui.png)



다음은 `menubar`의 `Open, Save`부터 구현한 코드다. Text Editor에서 했듯이 `actionOpen`과 `actionSave`를 `triggered` Signal을 통해 각각 `open_file`과 `save_file` 함수와 연결한다. `open_file`에서는 영상 파일을 열어서 클래스 멤버 변수 `self.src_img`에 저장 후 화면에 보여준다. `save_file`에서는 thresholding 결과 영상인 `self.res_img`를 파일로 저장한다.

```python
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

    def setup_ui(self):
        self.actionOpen.triggered.connect(self.open_file)
        self.actionSave.triggered.connect(self.save_file)

    def open_file(self):
        filename = QFileDialog.getOpenFileName(filter="JPG files (*.jpg)")
        filename = filename[0]
        print("open file:", filename)
        if not filename:
            return
        self.src_img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        cv2.imshow(self.src_title, self.src_img)
        cv2.waitKey(1)

    def save_file(self):
        filename = QFileDialog.getOpenFileName(filter="JPG files (*.jpg)", directory="../sample_imgs")
        filename = filename[0]
        print("save file:", filename)
        if not filename or self.res_img is None:
            return
        cv2.imwrite(filename, self.res_img)

def main():
    app = QApplication(sys.argv)
    editor = MyWindow()
    editor.show()
    app.exec_()

if __name__ == "__main__":
    main()
```



이제 조건에 따른 thresholding을 구현해보자. Text editor와는 다르게 이번에는 모든 radio button과 slider의 Signal을 하나의 함수로 연결한 후 그 함수에서 파라미터를 읽게 해보자. 다수의 radio button을 다룰 때 매번 if-else로 처리하기는 번거롭다. 이를 for문에서 처리할 수 있도록 radio button들을 해당 thresholding type과 함께 짝을 지어 `self.rb_dict`로 저장한다. 모든 radio button의 Slot 함수를 연결할 때도, `get_param()` 함수에서 선택된 radio button을 찾을 때도 다수의 if-else 대신 간단한 for문으로 해결할 수 있다.

```python
    def __init__(self):
        # .. 중략 ..
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
```

각각의 radio button을 눌러보고 slider를 움직이면서 효과를 느껴보자. 다음은 각 thresholding 방식을 비교한 그림이다.  

![thresh-all-types](/ian-lecture/assets/opencv-color/thresh-all-types.png)



이 그림은 MATLAB의 plot 기능을 모방한 `matplotlib`이라는 패키지를 통해 만든 것이다. 앞서 사용하지 않았던 `checkBox_all`을 클릭하면 모든 방식의 thresholding을 시도하여 한 창에 보여주도록 하였다. `matplotlib`은 아직 배우지 않았으니 다음 코드는 참고로만 봐두자.

```python
    def setup_ui(self):
        # .. 중략 ..
        self.checkBox_all_types.toggled.connect(self.show_all_types)

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
```

