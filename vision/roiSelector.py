import cv2
VIDEO_SOURCE = 2

cap = cv2.VideoCapture(VIDEO_SOURCE)
suc, image = cap.read()

cv2.imwrite("frame0.jpg", image)
img = cv2.imread("frame0.jpg")

r = cv2.selectROIs('ROI Selector', img, showCrosshair=False, fromCenter=False)

cap.release()
cv2.destroyAllWindows()