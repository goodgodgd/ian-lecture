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

newsize = (int(showimg.shape[1] / showimg.shape[0] * 300), 300)
showimg = cv2.resize(showimg, newsize, interpolation=cv2.INTER_NEAREST)
filename = os.path.join(IMG_PATH, "sungmo_thresh.png")
# cv2.imwrite(filename, showimg)
