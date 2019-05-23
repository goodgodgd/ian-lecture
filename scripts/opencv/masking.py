import cv2

poster = cv2.imread("../sample_imgs/infinity-war.jpg")
logo = cv2.imread("../sample_imgs/marvel-logo.jpg")
print("poster shape", poster.shape)
cv2.imshow("poster", poster)
cv2.imshow("mask", logo)
cv2.waitKey()

mask_gray = cv2.cvtColor(logo, cv2.COLOR_BGR2GRAY)
ret, mask_gray = cv2.threshold(mask_gray, 200, 255, cv2.THRESH_BINARY)
print("mask shape", mask_gray.shape)
cv2.imshow("mask", mask_gray)
cv2.waitKey()

mask_rgb = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)
print("mask shape", mask_rgb.shape)
cv2.imshow("mask", mask_rgb)
cv2.waitKey()

masked_poster = cv2.bitwise_and(poster, mask_rgb)
print("masked poster", masked_poster.shape)
cv2.imshow("poster", masked_poster)
cv2.waitKey()

mask_inv = cv2.bitwise_not(mask_rgb)
masked_logo = cv2.bitwise_and(logo, mask_inv)
masked_poster = cv2.add(masked_poster, masked_logo)
cv2.imshow("mask", masked_logo)
cv2.imshow("poster", masked_poster)
cv2.waitKey()
