import os
import cv2


class MouseEventHandler:
    def __init__(self, title, image):
        self.title = title
        self.image = image.copy()
        self.pt1 = (0, 0)

    def on_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 마우스를 눌렀을 때
            self.pt1 = (x, y)
            print("set pt1", self.pt1)
        elif event == cv2.EVENT_MOUSEMOVE:  # 마우스가 이동할 때
            pass
        elif event == cv2.EVENT_LBUTTONUP:  # 마우스 버튼을 올릴 때
            if self.pt1 == (x, y):
                return
            print("set pt2", (x, y))
            cv2.line(self.image, pt1=self.pt1, pt2=(x, y), color=(255, 0, 0))
            cv2.imshow(self.title, self.image)


def draw_line():
    IMG_PATH = "../sample_imgs"
    filename = os.path.join(IMG_PATH, "makrae.jpg")
    srcimg = cv2.imread(filename)
    window_name = "line_drawing"
    cv2.imshow(window_name, srcimg)

    mouse_hndl = MouseEventHandler(window_name, srcimg)
    # 반드시 imshow()나 namedWindow()로 윈도우를 먼저 만든 후 실행할 것
    cv2.setMouseCallback(window_name, mouse_hndl.on_mouse_event)
    cv2.waitKey()


if __name__ == "__main__":
    draw_line()
