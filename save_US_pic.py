import cv2
import time

cap = cv2.VideoCapture(0)
ret,frame = cap.read()
frame = cv2.resize(frame, (1920, 1080))
cv_image = frame[125:720, 645:1345]

cv2.imwrite('./US_pics/temp/{}.png'.format(time.strftime("%Y-%m-%d_%H-%M-%S")), cv_image)