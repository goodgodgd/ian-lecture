---
layout: post
title:  "[Python] Introduction to OpenCV"
date:   2019-05-11 09:00:01
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

설치할 때는 `opencv-python`이란 패키지로 설치하지만 코드에서 import 할 때는 `cv2`라는 이름으로 가져온다. 설치하는 패키지 이름과 import 하는 패키지 이름이 항상 같진 않다. 

# 2. Image I/O

## 2.1 Read Image

OpenCV를 설치하고 첫 번째로 해봐야 할 일은 영상을 화면에 띄워보는 것이다.  

> <<용어 정리>>
>
> 오늘날 "영상"과 "동영상" 두 단어가 혼용되고 있는데 수업에서는 정지된 사진(image)를 "영상"이라 부르고, 움직이는 영상(video)를 "동영상"이라고 부른다.

 간단한 코드로 이미지를 확인해보자.

```python
import cv2

image = cv2.imread("../sample_imgs/superson.jpg")
cv2.imshow("superson", image)
key = cv2.waitKey()
print("key in:", key)
cv2.destroyAllWindows()
```



![손흥민능욕.jpg](/ian-lecture/assets/opencv/superson-cap.jpg)

영상 입출력에 관련된 함수들을 살펴보자.



> **imread(filename, flag)**: 이미지 파일을 `flag`에 따라 읽는다.
>
> - cv2.IMREAD_COLOR: flag 기본 값, 일반적인 color 영상 읽기, alpha channel 무시
> - cv2.IMREAD_GRAYSCALE: 영상을 gray scale로 읽기
> - cv2.IMREAD_UNCHANGED: 영상을 alpha channel까지 포함하여 읽기
>
> **imshow(title, image)**: 제목과 영상을 입력하면 윈도우 창에 보여준다.
>
> **imwrite(filename, image)**: `image`를 `filename`이란 이름으로 저장한다.
>
> **waitKey(milisecond)**: 키보드 입력을 milisecond 동안 기다린다. 입력하지 않거나 0을 입력하면 무한대기하고 입력하면 그 시간만큼 기다린다. 입력이 들어오면 눌린 문자의 유니코드 값을 반환한다.
>
> **destroyAllWindows()**: 화면에 나타난 윈도우를 닫는다. 프로그램이 종료되면 자동으로 닫히지만 중간에 의도적으로 미리 닫고 싶을 때 쓴다.



## 2.2 Write Image

`imread()`에서 사용할 수 있는 세 가지 flag를 테스트하고 's'를 누르면 영상을 다른 이름으로 저장해보자.

```python
import os
import numpy as np
import cv2

# 파일명 만들기
IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "superson.jpg")
print("filename:", filename)
# 네 가지 형식으로 영상 불러오기
img_color = cv2.imread(filename, cv2.IMREAD_COLOR)
img_gray = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
img_unchange = cv2.imread(filename, cv2.IMREAD_UNCHANGED)
# 영상 보여주기
cv2.imshow("superson-color", img_color)
cv2.imshow("superson-gray", img_gray)
cv2.imshow("superson-unchange", img_unchange)
key = cv2.waitKey()
print("key in:", key, "==", chr(key))
# key가 's'이면 영상 저장
if key == ord('s'):
    filename = os.path.join(IMG_PATH, "superson-save.jpg")
    cv2.imwrite(filename, img_color)
cv2.destroyAllWindows()
```

참고로, 위 코드에서 `chr()`는 유니코드 숫자를 문자로 변환하는 함수고 `ord()`는 그 반대로 변환하는 함수다.

![손흥민능욕2.jpg](/ian-lecture/assets/opencv/superson-types.jpg)



# 3. Video I/O

## 3.1 Read Video

하나의 이미지 파일에서는 하나의 영상을 불러올 수 있지만 비디오 파일에서는 프레임 단위로 여러 이미지를 순서대로 받을 수 있다. 동영상을 읽을 때는 `cv2.VideoCapture` 객체를 생성해서 사용한다. 다음은 비디오 파일을 단순 재생하는 스크립트다.

```python
import os
import cv2
# 동영상 파일 열기
IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "endgame.mp4")
cap = cv2.VideoCapture(filename)
while 1:
    # 프레임 한 장 읽기
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('frame', frame)
    # 'q'를 누르면 종료
    if cv2.waitKey(33) == ord('q'):
        break
cap.release()
```

다음은 `cv2.VideoCapture`의 주요 함수다.



> **VideoCapture(filename)**: 객체 생성시 비디오 파일 이름을 입력하면 자동으로 비디오 파일을 열어(open)준다.
>
> **VideoCapture(camera_id)**: 객체 생성시 PC에 연결된 카메라 번호(주로 0)를 입력하면 카메라로부터 실시간 영상을 받을 수 있다.
>
> **isOpened()**: 파일이나 실시간 스트림이 정상적으로 열렸는지 확인하여 bool type으로 반환한다.
>
> **get(propId)**: 비디오 파일의 다양한 속성(크기, 시작위치 등)을 읽는다. `propId`로 입력한 속성을 반환한다. 비디오 속성의 목록은 [여기](https://docs.opencv.org/4.1.0/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d)서 확인할 수 있다.
>
> **set(propId, value)**: 비디오 파일의 다양한 속성을 수정한다. `propId`로 입력한 속성 값을 `value`로 바꾼다.
>
> **read()**: 한 프레임을 읽고 bool type으로 읽기의 성공 여부와 읽은 영상을 반환한다.



## 3.2 Write Video

동영상을 저장할 때는 `cv2.VideoWriter` 객체를 생성하여 사용한다. 주요 함수를 살펴보자.



> **VideoWriter(filename, fourcc, fps, frameSize)**: 객체 생성자
>
> - `filename`: 출력할 파일 이름 지정
> - `fourcc`: 인코딩 방식 지정, `cv2.VideoWriter_fourcc()` 함수로 생성
> - `fps`: 프레임 속도 지정
> - `frameSize`: 프레임 크기 지정
>
> **get(propId), set(propId, value)**:  `cv2.VideoCapture`와 마찬가지로 `cv2.VideoWriter` 객체의 속성을 읽고 쓴다. 속성은 [여기](<https://docs.opencv.org/4.1.0/d4/d15/group__videoio__flags__base.html#ga41c5cfa7859ae542b71b1d33bbd4d2b4>)서 확인할 수 있다.
>
> **write(image)**: `image`를 동영상의 한 프레임으로 저장한다.



`cv2.VideoWriter`를 활용하여 원본 동영상을 반으로 줄인 영상을 만들어보자.

```python
import os
import cv2

# 원본 동영상 파일 열기
IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "endgame.mp4")
cap = cv2.VideoCapture(filename)
if not cap.isOpened():
    raise FileNotFoundError()
print(f'get video property: width={cap.get(cv2.CAP_PROP_FRAME_WIDTH)}, '
      f'height={cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
cap.set(cv2.CAP_PROP_POS_MSEC, 20000)
# 저장할 동영상 파일 열기
filename = os.path.join(IMG_PATH, "endgame_rsz.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')    # *'DIVX'
new_size = (640, 360)
vout = cv2.VideoWriter(filename, fourcc, 30, new_size)
if not vout.isOpened():
    raise FileNotFoundError()

while 1:
    ret, frame = cap.read()
    if not ret:
        break
    # 프레임 영상 크기 조절
    frame_rsz = cv2.resize(frame, new_size)
    cv2.imshow('frame', frame_rsz)
    if cv2.waitKey(33) == ord('q'):
        break
    # 프레임 쓰기(저장)
    vout.write(frame_rsz)

cap.release()
vout.release()
```

![endgame.png](/ian-lecture/assets/opencv/endgame-cap.jpg)

# 4. Pixel Operation

지금까지는 파일을 읽고 쓰는 경험을 해 보았지만 실제로 영상처리에서 중요한 부분은 화소(pixel) 값을 내가 원하는 방식으로 바꾸거나, 단순하거 읽거나, 여러 화소 값을 해석해서 의미를 알아내는 일 등이다. `opencv-python`은 `numpy.ndarray` 타입으로 이미지를 다룬다. 지난 시간에 배웠던 numpy를 복습하며 영상의 화소 값을 다루는 연습을 해보자.  

## 4.1 속성 확인

간단하게 영상을 읽고 속성 확인 후 화면에 영상을 보여주는 스크립트이다. 

```python
import os
import cv2
import numpy as np

IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "jjang.jpg")
image = cv2.imread(filename)
# 영상 속성 확인
print(f"image info: shape={image.shape}, dtype={image.dtype}, size={image.size}")
# => image info: shape=(300, 326, 3), dtype=uint8, size=293400
cv2.imshow("jjangzeolmi", image)
cv2.waitKey()
```

속성을 해석해보자.

- **shape=(300, 326, 3)**: 영상의 높이 300, 너비 326, 채널이 BGR로 3채널 이라는 뜻이다.
- **dtype=uint8**: data type이 `uint8 = 8 bit unsigned int`라는 것으로 0~255 사이의 값을 가질 수 있다.
- **size=293400**: `size`는 array의 전체 원소 개수로 300X326X3=293,400 이다.



## 4.2 픽셀 값 읽고 쓰기

영상을 numpy로 표현하기 때문에 numpy의 indexing과 slicing을 이용할 수 있다. Indexing을 연습하기 위해 영상의 채널 별 평균을 구해보자.

```python
# loop over BGR channels
channels = {"blue": 0, "green": 1, "red": 2}
bgr_means = {}
for color, chn in channels.items():
    color_sum = 0
    for v in range(image.shape[0]):         # vertical axis
        for u in range(image.shape[1]):     # horizontal axis
            color_sum += image[v, u, chn]
    bgr_means[color] = color_sum / image[:, :, chn].size
print("BGR means:", bgr_means)
```

이번에는 영상의 값을 바꿔서 화면에 나오는 영상에 변화를 주자. 영상의 상하좌우에 각각 다른 색으로 여백을 만든다.

```python
image[:10, :, :] = 255      # 위에 흰 줄
image[-10:, :, :] = 0       # 아래에 검은 줄
image[:, :10, 0] = 255      # 왼쪽에 파란 줄
image[:, :10, 1:] = 0
image[:, -10:, 2] = 255     # 오른쪽에 빨간 줄
image[:, -10:, :2] = 0
cv2.imshow("jjangzeolmi", image)
cv2.waitKey()
```

![jjang-border.png](/ian-lecture/assets/opencv/jjang-border.jpg)



### 연습문제

영상 slicing을 통해 영상에 네모를 그려보세요.



# 5. Drawing Shapes

## 5.1 Basic Shapes

영상에서 도형을 그리기 위해 식을 계산하여 픽셀을 하나씩 쓸 필요는 없고 간단한 함수를 쓰면 된다. 도형 함수에서 공통 인자로 `img, color, thickness` 세 가지가 있는데 각각 그림이 그려질 영상, 도형의 색상, 선 두께를 의미한다.



> **line(img, start, end, color, thinckness)**: 영상 `img`에 시작점 `start`와 끝점 `end`를 잇는 선분을 그린다.
>
> **rectangle(img, pt1, pt2, color, thickness)**: 영상 `img`에 `pt1`과 `pt2`를 반대편 모서리로 하는 직사각형을 그린다.
>
> **circle(img, center, raidus, color, thickness)**: 영상 `img`에 `center`를 중심으로 반지금 `radius`인 원을 그린다.
>
> **putText(img, text, org, fontFace, fontScale, color, thickness)**: 영상 `img`에 `text`를 `org`(origin) 위치에 쓴다. `org`는 text의 bottom-left corner 좌표다. `fontFace`는 폰트 타입으로 [여기](<https://docs.opencv.org/4.1.0/d6/d6e/group__imgproc__draw.html#ga0f9314ea6e35f99bb23f29567fc16e11>)서 고를 수 있다. 

이 외에도 OpenCV에서는 `arrowedLine(), ellipse(), drawMarker(), fillPoly()` 등의 다양한 drawing 함수를 제공한다. 자세한 내용은 [이곳](<https://docs.opencv.org/4.1.0/d6/d6e/group__imgproc__draw.html>)에서 확인할 수 있다.  

그럼 이제 실제 그림을 그려보자. 

```python
import os
import cv2

IMG_PATH = "../sample_imgs"
filename = os.path.join(IMG_PATH, "makrae.jpg")
srcimg = cv2.imread(filename)
shapeimg = srcimg.copy()
# 도형 그리기
cv2.line(shapeimg, pt1=(50, 110), pt2=(105, 110), color=(255, 0, 0), thickness=2)
cv2.circle(shapeimg, center=(33, 150), radius=20, color=(0, 0, 255), thickness=2)
cv2.rectangle(shapeimg, pt1=(160, 20), pt2=(340, 240), color=(0, 255, 0), thickness=1)
cv2.putText(shapeimg, text="Korea Grandma", org=(360, 100),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(50, 50, 50))
# 보여주기
cv2.imshow("original", srcimg)
cv2.imshow("draw shape", shapeimg)
cv2.waitKey()
```

![makrae-shape.png](/ian-lecture/assets/opencv/makrae-shape.jpg)



## 5.2. Callback Function

위와 같이 영상 좌표를 직접 입력하는 것은 여러번 확인하면서 고쳐야 하므로 매우 번거로운 일이다. OpenCV에서는 다음 함수를 이용하여 영상 윈도우로 들어오는 사용자 입력(마우스, 키보드)을 처리할 수 있는 callback 함수를 등록할 수 있다.  

> **cv2.setMouseCallback(windowName, callback, param=None)** 
>
> - **windowName**: callback을 등록할 window 이름, `cv2.namedWindow()`나 `cv2.imshow()` 함수에서 window 이름을 지정한다.
> - **callback**: 이벤트 발생시 실행 될 함수
> - **param**: `callback` 함수로 전달될 입력 인자

만들어서 등록해야 할 callback 함수의 프로토 타입은 다음과 같다.

> on_mouse_event(event, x, y, flags, param)
>
> - **event**: 발생한 마우스 이벤트 종류다. 마우스 이벤트는 [이곳](<https://docs.opencv.org/4.1.0/d7/dfc/group__highgui.html#ga927593befdddc7e7013602bca9b079b0>)에 정의되어있다.
> - **x, y**: 이벤트가 발생한 좌표다.
> - **flags**: 현재의 상태를 나타내는 플래그로서 [이곳](<https://docs.opencv.org/4.1.0/d7/dfc/group__highgui.html#gaab4dc057947f70058c80626c9f1c25ce>)에 정의되어있다.
> - **param**: `cv2.setMouseCallback`에서 전달한 파라미터



여기서는 마우스로 드래그하여 선분을 그리는 예제를 만들어보자. OpenCV Tutorial 에서는 callback 함수의 특성상 전역 변수를 사용하였지만 전역 변수는 가급적 지양해야 하므로 callback을 처리하는 클래스를 만들었다.

```python
import os
import cv2

class MouseEventHandler:
    def __init__(self, title, image):
        self.title = title
        self.image = image.copy()
        self.pt1 = (0, 0)

    def on_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 마우스를 눌렀을 때
            self.pt1 = (x, y)
            print("set pt1", self.pt1)
        elif event == cv2.EVENT_MOUSEMOVE:  # 마우스가 이동할 때
            pass
        elif event == cv2.EVENT_LBUTTONUP:  # 마우스 버튼을 올릴 때
            if self.pt1 == (x, y):
                return
            print("set pt2", (x, y))
            cv2.line(self.image, pt1=self.pt1, pt2=(x, y), color=(255, 0, 0))
            cv2.imshow(self.title, self.image)

def draw_line():
    IMG_PATH = "../sample_imgs"
    filename = os.path.join(IMG_PATH, "makrae.jpg")
    srcimg = cv2.imread(filename)
    window_name = "line_drawing"
    cv2.imshow(window_name, srcimg)

    mouse_hndl = MouseEventHandler(window_name, srcimg)
    # 반드시 imshow()나 namedWindow()로 윈도우를 먼저 만든 후 실행할 것
    cv2.setMouseCallback(window_name, mouse_hndl.on_mouse_event)
    cv2.waitKey()

if __name__ == "__main__":
    draw_line()
```

Callback 함수에 값을 전달하기 위해 전역 변수로 선언했어야 할 변수들을 클래스 멤버 변수로 선언하고 이벤트를 처리하는 `on_mouse_event()` 함수도 멤버 함수로 선언하여 사용했다. ~~근데 결과를 보니 아까 코드로 그렸던게 더 예쁜듯~~

![makrae-callback.png](/ian-lecture/assets/opencv/makrae-callback.jpg)



*그런데 말입니다.* 아래 줄을 자세히 보자. 뭔가 이상하지 않은가?

```python
cv2.setMouseCallback(window_name, mouse_hndl.on_mouse_event)
```

Callback 함수를 등록할 때 C++ 같은 경우에는 함수 포인터를 넣던지 Lambda 함수를 만들어 넣던지 하는데 여기서는 심지어 객체에 소속된 함수인데 그냥 막 넣어도 잘 작동했다. 이건 파이썬의 함수가 기본적으로 **일급 함수**기 때문이다.

> 일급 함수: 쉽게 말해서 객체처럼 취급 가능한 함수다. 일급 함수의 조건은 다음과 같다.
>
> - 함수를 변수나 데이터 구조안에 담을 수 있다.
> - 어떤 함수를 다른 함수의 입력 인자로 전달 가능
> - 함수에서 함수를 반환 가능

따라서 파이썬에서는 다음과 같은 코드가 가능하다.

```python
def foo():
    print("foo")

class bar:
    def bar_func(self):
        print("bar")

def spam(f1, f2):
    f1()
    f2()

eggs = bar()
spam(foo, eggs.bar_func)
```

### 연습문제

마우스 이벤트를 이용하여 영상에 마우스로 자유로운 곡선을 그려보세요.

