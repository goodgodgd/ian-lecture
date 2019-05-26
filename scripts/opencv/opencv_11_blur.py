import cv2
import numpy as np
import show_imgs as si
IMG_PATH = "../sample_imgs"


def blur():
    image = cv2.imread(IMG_PATH + "/yumi-cells.jpg")
    kernel_sizes = [(3, 3), (5, 5), (7, 7), (7, 1), (1, 7)]
    filter_imgs = {"original": image}
    blur_imgs = {"original": image}
    for ksize in kernel_sizes:
        title = f"ksize: {ksize}"
        kernel = np.ones(ksize)
        kernel /= kernel.size
        filter_imgs[title] = cv2.filter2D(image, -1, kernel)
        blur_imgs[title] = cv2.blur(image, ksize)
    resimg = si.show_imgs(filter_imgs, "cv2.filter2D", 3)
    resimg = si.show_imgs(blur_imgs, "cv2.blur", 3)
    # cv2.imwrite(IMG_PATH + "/yumi-blur.jpg", resimg)


def gaussian():
    image = cv2.imread(IMG_PATH + "/yumi-cells.jpg")
    kernel_size = (5, 5)
    blur_imgs = {}
    blur_imgs["original"] = image
    blur_imgs["blur"] = cv2.blur(image, kernel_size)
    blur_imgs["GaussianBlur"] = cv2.GaussianBlur(image, kernel_size, 0)
    result_img = si.show_imgs(blur_imgs, "GaussianBlur", 3, 1000)
    # cv2.imwrite(IMG_PATH + "/yumi-gaussian.jpg", result_img)


def median():
    image = cv2.imread(IMG_PATH + "/saltnpepper.jpg")
    kernel_size = (5, 5)
    median_imgs = dict()
    median_imgs["original"] = image
    median_imgs[f"gaussian {kernel_size}"] = cv2.GaussianBlur(image, kernel_size, 0)
    median_imgs[f"median {kernel_size[0]}"] = cv2.medianBlur(image, kernel_size[0])
    result_img = si.show_imgs(median_imgs, "Median Filter", 3)
    # cv2.imwrite(IMG_PATH + "/snp-median.jpg", result_img)

    image = cv2.imread(IMG_PATH + "/ann.jpg")
    median_imgs = dict()
    median_imgs["original"] = image
    median_imgs["median (3)"] = cv2.medianBlur(image, 3)
    median_imgs["median (5)"] = cv2.medianBlur(image, 5)
    result_img = si.show_imgs(median_imgs, "Median Filter", 3)
    # cv2.imwrite(IMG_PATH + "/ann-median.jpg", result_img)


def bilateral():
    image = cv2.imread(IMG_PATH + "/road.jpg")
    kernel_size = (5, 5)
    blur_imgs = {}
    blur_imgs["original"] = image
    blur_imgs["gaussian"] = cv2.GaussianBlur(image, kernel_size, 0)
    blur_imgs["bilateral (5,50,50)"] = cv2.bilateralFilter(image, 5, 50, 50)
    blur_imgs["bilateral (5,150,150)"] = cv2.bilateralFilter(image, 5, 150, 150)
    result_img = si.show_imgs(blur_imgs, "Bilateral Filter", 2)
    cv2.imwrite(IMG_PATH + "/road-bilateral.jpg", result_img)


if __name__ == "__main__":
    # blur()
    # gaussian()
    # median()
    bilateral()
