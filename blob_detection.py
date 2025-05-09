import cv2
import numpy as np

# Load image in grayscale
image = cv2.imread('/home/matija/Downloads/target_3 1.jpg')
cv2.imshow('orig',image)



params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 45
params.maxThreshold = 255

# Filter by Area
params.filterByArea = True
params.maxArea = 120

# Filter by Circularity
params.filterByCircularity = False

# Filter by Convexity
params.filterByConvexity = False

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

params.filterByColor = True
params.blobColor = 255


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#thresh = cv2.adaptiveThreshold(image,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,5,2)
thresh = cv2.threshold(gray,45,255,cv2.THRESH_BINARY)[1]
cv2.imshow('tresh',thresh)


# Detect blobs
keypoints = detector.detect(image)

blank = np.zeros(np.shape(gray))

for keypoint in keypoints:
    x = int(keypoint.pt[0])
    y = int(keypoint.pt[1])
    blank = cv2.circle(blank,(x,y),3,255,-1)
#cv2.imwrite(self.img_path + '/output_images/slice_{:.5f}.jpg'.format(angle),blank)
cv2.imshow('blobs',blank)
cv2.waitKey(0)