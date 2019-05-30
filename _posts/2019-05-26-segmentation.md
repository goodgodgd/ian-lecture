---
layout: post
title:  "[Python] Segmentation"
date:   2019-05-26 09:00:01
categories: 2019-1-systprog
---



# 1. Hough Transform

**허프 변환(Hough Transform)**은 영상에서 직선이나 원과 같은 모양을 찾는 방법이다. 먼저 영상에서 캐니 엣지로 외곽선을 추출하고 엣지로 검출된 픽셀 좌표들을 **허프 공간(Hough Space)**으로 변환한다. 허프 공간은 찾고자 하는 도형의 파라미터 공간(Parameter Space)이며 이곳에서 밀도가 높은 곳의 파라미터를 찾으면 해당 도형을 픽셀 좌표계에서 찾을 수 있다. 가장 대표적인 직선 검출을 예로 들면 아래 왼쪽 그림처럼 영상에서 직선은 $(\rho, \theta)$ 두 개의 파라미터로 결정할 수 있다. 엣지로 검출된 선분 위의 한 픽셀을 지날수 있는 모든 직선의 파라미터를 허프 공간에 표시하면 오른쪽 그림과 같이 곡선이 그려진다. 모든 엣지의 픽셀에 대해서 허프 공간의 곡선을 그려본다. 허프 공간을 일정 간격의 그리드(grid)로 나눈 후 각 그리드를 지나는 선분의 개수가 가장 높은 그리드의 파라미터가 우리가 찾는 선분의 파라미터가 된다. 이때 각 그리드를 지나는 선분의 개수를 **vote**라고 한다. 원을 찾을때도 마찬가지로 원을 $(x,y,r)$ 세 개의 파라미터로 특정할 수 있으므로 3차원 허프 공간을 만들고 엣지 위의 픽셀들을 허프 공간으로 변환하면 된다.

![hough-transform1](/ian-lecture/assets/opencv-segment/hough-transform1.jpg)

![hough-transform2](/ian-lecture/assets/opencv-segment/hough-transform2.png)



## 1.1 Hough Line Transform

OpenCV에서는 허프 직선 변환을 구현한 `cv2.HoughLines()`와 확률적인 방법으로 직선 검출의 속도를 높인 `cv2.HoughLinesP()`라는 두 개의 함수를 제공한다.

> lines = **cv2.HoughLines**(image, rho, theta, threshold[, lines, srm, stm, min_theta, max_theta])
>
> - image: 8-bit, single channel binary source image. 주로 원본 영상에 가우시안 블러와 캐니 엣지를 순서대로 적용한 영상을 입력 영상으로 쓴다.
> - rho: 직선과 원점과의 거리 측정 해상도, 허프 공간을 나누는 단위, 작을수록 정확히 측정할 수 있지만 입력 영상의 노이즈에 취약해진다.
> - theta: 원점에서 직선에 내린 수선의 발의 각도 측정 해상도, 허프 공간을 나누는 단위, 작을수록 정확히 측정할 수 있지만 입력 영상의 노이즈에 취약해진다.
> - threshold: 직선으로 판단한 최소한의 vote 개수, 작게 주면 검출 개수가 증가하지만 정확도가 감소, 높게 주면 검출 개수는 줄어들지만 확실한 직선만 검출
> - lines: 검출 결과, 1 1N x 1 x 2 크기의 배열 $(\rho, \theta)$
> - srn, stn: 허프 변환을 목표 해상도에 바로 적용하는 것은 많은 연산이 필요하므로 낮은 해상도에서 점점 높은 해상도로 올리면서 찾는다. 이때 단계별 거리와 각도 해상도 갱신 비율을 srn, stn 으로 지정한다.
> - min_theta, max_theta: 검출을 위해 사용할 최소, 최대 각도

> lines = **cv2.HoughLinesP**(image, rho, theta, threshold[, lines, minLineLength, maxLineGap])
>
> - image, rho, theta, threshold는 위와 동일
> - lines: 검출 결과, N x 1 x 4 크기의 배열로 시작점과 끝점 좌표 표현 $(x_1, y_1, x_2, y_2)$
> - minLineLength: 선으로 인정할 최소 길이
> - maxLineGap: 하나의 직선이 두 개 이상의 선분으로 끊어져 있을 때 얼마나 끊어진 간격까지 연결해서 하나의 선분으로 검출할 것인지 결정

두 함수 중에서 `cv2.HoughLinesP`가 속도가 빠를 뿐더러 결과도 사용자가 쓰기 쉬운 끝점 좌표로 나오고 입력 인자도 선분의 조건을 직관적으로 제한할 수 있기 때문에 많이 쓰인다. 다만 선 검출이 `cv2.HoughLinesP()`에 비해서는 적게 되므로 threshold를 상대적으로 낮게 지정해야 한다. 다음은 `cv2.HoughLinesP` 함수를 이용해 책장의 선분을 검출하는 예시다. 결과로 나온 배열을 사용할 때 주의할 점은 크기가 (Nx4)가 아니라 (Nx1x4)라는 것이다.

```python
import cv2
import numpy as np
import show_imgs as si
IMG_PATH = "../sample_imgs"

def hough_lines():
    img_names = [IMG_PATH + f"/bookshelf{i+1}.jpg" for i in range(3)]
    images = {}
    for i, name in enumerate(img_names):
        images[f"srcimg{i+1}"] = cv2.imread(name, cv2.IMREAD_COLOR)

    canny_edges = {}
    for key, img in images.items():
        grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurimg = cv2.GaussianBlur(grayimg, (3, 3), 0)
        # blurimg = cv2.GaussianBlur(blurimg, (3, 3), 0)
        canny_edges[key.replace("srcimg", "canny")] = cv2.Canny(blurimg, 100, 200)

    hough_results = {}
    for key, canny_img in canny_edges.items():
        lines = cv2.HoughLi(canny_img, 1, np.pi/180, 50, None, 50, 10)
        result = images[key.replace("canny", "srcimg")].copy()
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(result, (x1,y1), (x2,y2), (0,0,255), 1)
        hough_results[key.replace("canny", "houghline")] = result

    images.update(canny_edges)
    images.update(hough_results)
    for key, img in images.items():
        if len(img.shape) == 2:
            images[key] = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    result_img = si.show_imgs(images, "hough lines", 3, 1000)

if __name__ == "__main__":
    hough_lines()
```

아래는 결과 영상이다. `threshold, minLineLength, maxLineGap` 세 개의 파라미터를 조절하면서 어떤 효과가 나는지 확인해보고 가능하면 GUI로 조절할 수 있게 구현해보자.

![houghlines](/ian-lecture/assets/opencv-segment/houghlines.jpg)



## 1.2 Hough Circle Transform

OpenCV는 Hough Circle Transform을 구현한 `cv2.HoughCircles()` 함수를 제공한다.

> circles - **cv2.HoughCircles**(image, method, dp, minDist[, circles, param1, param2, minRadius, maxRadius])
>
> - image: 8-bit, single-channel, grayscale input image, HoughLines()와는 다르게 엣지 영상을 넣는 것이 아니라 단일 채널 원본 영상을 넣어준다.
> - method: 현재는 cv2.HOUGH_GRADIENT 만 구현이 되어있다.
> - dp: 허프 공간에서의 그리드 크기, dp=1이면 입력 영상의 픽셀 해상도와 같아진다.
> - minDist: 검출 원들 사이의 최소 거리
> - param1: 캐니 엣지에서 higher threshold 값, 그 절반을 lower threshold로 사용
> - param2: 원으로 판단하는 최소 vote 수
> - minRadius, maxRadius: 최소, 최대 반지름, 적당한 반지름 범위를 입력하면 잘못된 원 검출을 크게 줄일 수 있다.

다음은 `cv2.HoughCircles()` 함수를 이용해 두더지 잡기 게임에서 두더지 구멍을 찾는 예시다. 캐니 엣지는 함수 내부적으로 사용하기 때문에 따로 구해볼 필요는 없지만 내부 사정을 이해하기 위해 같은 파라미터를 넣고 캐니 엣지를 계산하였다. 노이즈를 줄여주기 위한 전처리로 여러가지 블러링 필터를 써봤는데 엣지를 살려주는 `cv2.bilateralFilter()`가 가장 나은 성능을 보여주었다.

```python
def hough_circles():
    img_names = [IMG_PATH + f"/mole{i+1}.jpg" for i in range(3)]
    images = {}
    for i, name in enumerate(img_names):
        images[f"srcimg{i+1}"] = cv2.imread(name, cv2.IMREAD_COLOR)

    canny_edges = {}
    for key, img in images.items():
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 5, 100, 100)
        canny_edges[key.replace("srcimg", "canny")] = cv2.Canny(gray, 100, 200)

    hough_results = {}
    for key, srcimg in images.items():
        gray = cv2.cvtColor(srcimg, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 5, 100, 100)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 30, None,
                                   param1=200, param2=50, maxRadius=60)
        if circles is not None:
            circles = np.around(circles).astype(np.uint16)
            circles = circles[0]
            result = srcimg.copy()
            print("circles", circles)
            for circle in circles:
                result = cv2.circle(result, (circle[0], circle[1]), circle[2], (0,0,255), 2)
            hough_results[key.replace("srcimg", "houghcircle")] = result

    images.update(canny_edges)
    images.update(hough_results)
    for key, img in images.items():
        if len(img.shape) == 2:
            images[key] = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    result_img = si.show_imgs(images, "hough lines", 3, 1000)
```

결과를 보면 원이 밖에 걸쳐있는 것을 제외하고는 대부분의 두더지 구멍을 찾았다. 물론 원의 위치가 사람의 생각과는 조금 다르지만 이는 원을 정면이 아닌 사선 방향에서 찍어서 엣지가 타원으로 나오기 때문이다. 캐니 엣지를 보면 이해가 될 것이다.

![houghcircles](/ian-lecture/assets/opencv-segment/houghcircles.jpg)



# 2. FloodFill









# 3. Watershed



# 4. GrabCut