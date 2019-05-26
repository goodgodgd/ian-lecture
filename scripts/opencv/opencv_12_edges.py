import cv2
import numpy as np
import show_imgs as si
IMG_PATH = "../sample_imgs"


def sobel():
    image = cv2.imread(IMG_PATH + "/yumi-cells.jpg", cv2.IMREAD_GRAYSCALE)
    sobel_imgs = {"original": image}
    sobel_imgs["Sobel dx"] = cv2.Sobel(image, -1, 1, 0, 3)
    sobel_imgs["Sobel dy"] = cv2.Sobel(image, -1, 0, 1, 3)
    sobel_imgs["Sobel dx+dy"] = cv2.add(sobel_imgs["Sobel dx"], sobel_imgs["Sobel dy"])
    sobel_imgs["Scharr dx"] = cv2.Scharr(image, -1, 1, 0)
    sobel_imgs["Scharr dy"] = cv2.Scharr(image, -1, 0, 1)
    result_img = si.show_imgs(sobel_imgs, "Sobel & Scharr", 3)
    # cv2.imwrite(IMG_PATH + "/yumi-edges.jpg", result_img)


def laplacian():
    image = cv2.imread(IMG_PATH + "/yumi-cells.jpg", cv2.IMREAD_GRAYSCALE)
    lapla_imgs = {"original": image}
    sobel_dx = cv2.Sobel(image, -1, 1, 0, 3)
    sobel_dy = cv2.Sobel(image, -1, 0, 1, 3)
    lapla_imgs["Sobel dx+dy"] = cv2.add(sobel_dx, sobel_dy)
    lapla_imgs["Laplacian"] = cv2.Laplacian(image, -1)
    result_img = si.show_imgs(lapla_imgs, "Laplacian", 3)
    # cv2.imwrite(IMG_PATH + "/yumi-laplacian.jpg", result_img)


def canny():
    image = cv2.imread(IMG_PATH + "/arya.jpg", cv2.IMREAD_GRAYSCALE)
    canny_imgs = {"original": image}
    canny_imgs["Laplacian"] = cv2.Laplacian(image, -1, scale=2)
    canny_imgs["Canny (100, 200)"] = cv2.Canny(image, 100, 200)
    canny_imgs["Canny (200, 255)"] = cv2.Canny(image, 150, 255)
    result_img = si.show_imgs(canny_imgs, "Canny", 2)
    # cv2.imwrite(IMG_PATH + "/arya-canny.jpg", result_img)


if __name__ == "__main__":
    # sobel()
    # laplacian()
    canny()
