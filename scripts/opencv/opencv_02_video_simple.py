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
