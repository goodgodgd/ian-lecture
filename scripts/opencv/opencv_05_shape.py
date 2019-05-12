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
