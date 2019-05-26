import cv2
import numpy as np
import show_imgs as si
IMG_PATH = "../sample_imgs"


def erode_and_dilate():
    srcimg = cv2.imread(IMG_PATH + "/marvel.jpg", cv2.IMREAD_GRAYSCALE)
    srcimg = salt_and_pepper_noise(srcimg)
    # erode and dilate, and then show them
    images = {"original": srcimg}
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    images["erode"] = cv2.erode(srcimg, kernel)
    images["dilate"] = cv2.dilate(srcimg, kernel)
    result_img = si.show_imgs(images, "Erode and Dilate", 1)
    # cv2.imwrite(IMG_PATH + "/erode-dilate.jpg", result_img)


def salt_and_pepper_noise(image):
    towhite = np.unravel_index(np.random.randint(0, image.size, 100), image.shape)
    toblack = np.unravel_index(np.random.randint(0, image.size, 100), image.shape)
    image[towhite] = 255
    image[toblack] = 0
    return image


def morphologies():
    srcimg = cv2.imread(IMG_PATH + "/marvel.jpg", cv2.IMREAD_GRAYSCALE)
    srcimg = salt_and_pepper_noise(srcimg)
    # erode and dilate, and then show them
    images = {"original": srcimg}
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    images["opening"] = cv2.morphologyEx(srcimg, cv2.MORPH_OPEN, kernel)
    images["closing"] = cv2.morphologyEx(srcimg, cv2.MORPH_CLOSE, kernel)
    images["gradient"] = cv2.morphologyEx(srcimg, cv2.MORPH_GRADIENT, kernel)
    images["tophat"] = cv2.morphologyEx(srcimg, cv2.MORPH_TOPHAT, kernel)
    images["blackhat"] = cv2.morphologyEx(srcimg, cv2.MORPH_BLACKHAT, kernel)
    result_img = si.show_imgs(images, "Morphology Ops", 2)
    # cv2.imwrite(IMG_PATH + "/morphologies.jpg", result_img)


if __name__ == "__main__":
    erode_and_dilate()
    morphologies()
