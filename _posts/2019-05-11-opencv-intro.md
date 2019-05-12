---
layout: post
title:  "[Python] Introduction to OpenCV"
date:   2019-04-30 09:00:01
categories: 2019-1-systprog
---



# 1. About OpenCV

OpenCV (Open Source Computer Vision Library) is an open source computer vision and machine learning software library. OpenCV was built to provide a common infrastructure for computer vision applications and to accelerate the use of machine perception in the commercial products.  

OpenCV는 영상(Image)를 다루는 라이브러리로서 현재 세계적으로 독보적이고 거의 독점직인 위치를 가지고 있다. Open source이고 상업적으로도 자유롭게 쓸 수 있는 BSD-License인데다 최적화된 방대한 알고리즘을 제공하니 다른 라이브러리를 쓸 이유가 없다. 이제는 OpenCV 없이 이미지를 다루는 방법을 찾기가 어려울 정도로 보편화 되어있다. Python에서는 원래 `scikit-image`라는 패키지가 영상처리에 많이 쓰였지만 OpenCV가 Python Binding을 제공하면서부터 OpenCV 이용자가 많아졌다.

OpenCV는 2500개 이상의 최적화된 알고리즘을 제공하여 많은 사람들이 사용하는 정형화된 알고리즘은 다 있다고 보면 된다. 단순히 영상처리 뿐만 아니라 얼굴인식, 동작인식, 객체추적, 3차원 형상 복원 등 다양한 머신러닝 알고리즘도 지속적으로 추가되고 있다. 영상을 이용하는 논문 중에 유명하고 많이 쓰이는 알고리즘은 거의다 Open source로 개발되어 올라온다. 그래서 OpenCV는 버전 주기가 짧은 편이며 거의 한 달에 한번씩 새로운 버전이 올라온다.  

OpenCV는 원래 1999년 인텔의 러시아 팀에서 개발이 되었다. (OpenCV 아버지 Gary Bradski) 첫 번째 alpha 버전은 2000년에 나오고 중간에 beta 버전들이 나온 후, 2006년에 OpenCV 1.0 나오게 됐다. (내가 처음 OpenCV를 접한 것도 학부 4학년 2008년 이었는데 그때 이미 한글책도 나오고 보편적으로 쓰이고 있었다.)  

중간에 다른 회사에서 관리하다가 이제는 [opencv.org](<https://opencv.org/>)라는 단체에서 "관리"한다. 여기서 관리라 함은 여러 개발자들이 올려준 소스들을 검증하고 다음 Release에 넣을지 등을 결정한다는 것이고 개발은 수많은 자원자들과 이를 지원하는 회사들에 의해 진행 된다. 2019년 5월 현재는 OpenCV 4.1이 최신 버전이며 pypi에 올라온 `opencv-python` 패키지도 최신 버전을 제공한다.  



### OpenCV 설치

```bash
pip install opencv-python
```



# 2. Image I/O

OpenCV를 설치하고 첫 번째로 해봐야 할 일은 영상을 화면에 띄워보는 것이다.  

> <<용어 정리>>
>
> 오늘날 "영상"과 "동영상" 두 단어가 혼용되고 있는데 수업에서는 정지된 사진(image)를 "영상"이라 부르고, 움직이는 영상(video)를 "동영상"이라고 부른다.

설치할 때는 `opencv-python`이란 패키지로 설치했지만 코드에서 import 할 때는 `cv2`라는 이름으로 가져온다. 설치하는 패키지 이름과 import 하는 패키지 이름이 항상 같진 않다.  간단한 코드로 이미지를 확인해보자.

```python
import cv2

image = cv2.imread("../../assets/opencv/superson.jpg")
cv2.imshow("superson", image)
key = cv2.waitKey()
print("key in:", key)
cv2.destroyAllWindows()
```



![손흥민능욕.jpg](/ian-lecture/assets/opencv/superson-cap.jpg)

위 코드에서 사용된 함수들을 살펴보자.

- **imread(filename, flag)**: 이미지 파일을 `flag`에 따라 읽는다.
  - cv2.IMREAD_COLOR: flag 기본 값, 일반적인 color 영상 읽기, alpha channel 무시
  - cv2.IMREAD_GRAYSCALE: 영상을 gray scale로 읽기
  - cv2.IMREAD_UNCHANGED: 영상을 alpha channel까지 포함하여 읽기
- **imshow(title, image)**: 제목과 영상을 입력하면 윈도우 창에 보여준다.
- **waitKey(milisecond)**: 키보드 입력을 milisecond 동안 기다린다. 입력하지 않거나 0을 입력하면 무한대기하고 입력하면 그 시간만큼 기다린다. 입력이 들어오면 눌린 문자의 아스키 코드를 반환한다.
- **destroyAllWindows()**: 화면에 나타난 윈도우를 닫는다. 프로그램이 종료되면 자동으로 닫히지만 중간에 의도적으로 미리 닫고 싶을 때 쓴다.



`imread()`에서 사용할 수 있는 세 가지 flag를 테스트해보자.

```python
import os
import numpy as np
import cv2

# 파일명 만들기
IMG_PATH = "../../assets/opencv/"
filename = os.path.join(IMG_PATH, "superson.jpg")
print("filename:", filename)
# 세 가지 형식으로 영상 불러오기
img_color = cv2.imread(filename, cv2.IMREAD_COLOR)
img_gray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
img_unchange = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
# 영상 보여주기
cv2.imshow("superson-color", img_color)
cv2.imshow("superson-gray", img_gray)
cv2.imshow("superson-unchange", img_unchange)
key = cv2.waitKey()
cv2.destroyAllWindows()
```



![손흥민능욕2.jpg](/ian-lecture/assets/opencv/superson-types.jpg)



# 3. Video I/O



# 4. Pixel Operation



# 5. Drawing Shapes



다음에

# 6. Color Space

