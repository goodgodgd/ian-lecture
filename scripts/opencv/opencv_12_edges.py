import cv2
import numpy as np
import show_imgs as si
IMG_PATH = "../sample_imgs"


def sobel():
    image = cv2.imread(IMG_PATH + "/yumi-cells.jpg", cv2.IMREAD_GRAYSCALE)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    sobel_imgs = {"original": image}
    sobel_imgs["Sobel dx"] = cv2.Sobel(image, -1, 1, 0, 3)
    sobel_imgs["Sobel dy"] = cv2.Sobel(image, -1, 0, 1, 3)
    sobel_imgs["Sobel dx+dy"] = cv2.add(sobel_imgs["Sobel dx"], sobel_imgs["Sobel dy"])
    sobel_imgs["Scharr dx"] = cv2.Scharr(image, -1, 1, 0)
    sobel_imgs["Scharr dy"] = cv2.Scharr(image, -1, 0, 1)
    result_img = si.show_imgs(sobel_imgs, "Sobel & Scharr", 3)
    cv2.imwrite(IMG_PATH + "/yumi-edges.jpg", result_img)


if __name__ == "__main__":
    sobel()
