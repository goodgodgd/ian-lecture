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