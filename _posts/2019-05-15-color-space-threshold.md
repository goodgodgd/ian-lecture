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



이번에도 `cvtColor()` 함수를 써서 color space를 변환하였다. BGR에서 HSV로 변환하는 코드는 `cv2.COLOR_BGR2HSV` 이다.

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

왼쪽의 톰 홀랜드는 색상이 명확하고 배경이 단순하기 때문에 거의 정확하게 얼굴과 몸이 구분된 것을 볼 수 있다. 반면 오른쪽의 토비 맥과이어는 색상이 어두워서 색상의 해상도가 좋지 않고 배경에도 파란색이 들어가 있어서 segmentation의 정확도가 떨어진다. 여기서 중요한 것은 **양쪽의 명암이 크게 다른데도 색상을 기준으로 비슷한 결과**를 냈다는 것이다.  

이러한 segmentation을 BGR에서도 구현할 수 있을까? BGR에서 빨간 영역을 추출하려면 R이 높고 BG는 낮은 영역을 찾아야 할 것 이다. 하지만 R값 자체도 명암에 따라 크게 달라지고 토미 맥과이어처럼 어두운 영역까지 추출하려고 한다면 범위가 너무 커져서 다른 영역까지 함께 추출될 것이다. 따라서 BGR은 화소 값 기반 segmentation에  적합한 color space가 아니다.  



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

