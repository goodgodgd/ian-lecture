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
        lines = cv2.HoughLinesP(canny_img, 1, np.pi/180, 50, None, 50, 10)
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

    result_img = si.show_imgs(images, "hough lines", 3, 1200)
    cv2.imwrite(IMG_PATH + "/houghlines.jpg", result_img)


def hough_circles():
    img_names = [IMG_PATH + f"/mole{i+1}.jpg" for i in range(3)]
    images = {}
    for i, name in enumerate(img_names):
        images[f"srcimg{i+1}"] = cv2.imread(name, cv2.IMREAD_COLOR)

    canny_edges = {}
    for key, img in images.items():
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        canny_edges[key.replace("srcimg", "canny")] = cv2.Canny(gray, 100, 200)

    hough_results = {}
    for key, srcimg in images.items():
        gray = cv2.cvtColor(srcimg, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
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
    result_img = si.show_imgs(images, "hough lines", 3, 1200)
    cv2.imwrite(IMG_PATH + "/houghcircles.jpg", result_img)


if __name__ == "__main__":
    # hough_lines()
    hough_circles()
