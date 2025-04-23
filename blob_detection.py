import cv2
import numpy as np

# Load image in grayscale
image = cv2.imread('C:\Files\Polaris_integration\data\image_006.png')



# Set up the detector with default parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 180
params.maxThreshold = 255

# Filter by Area
params.filterByArea = True
params.minArea = 10
params.maxArea = 500

# Filter by Circularity
params.filterByCircularity = False

# Filter by Convexity
params.filterByConvexity = False

# Filter by Inertia
params.filterByInertia = False

params.filterByColor = True
params.blobColor = 255


# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)
# _,thresh = cv2.threshold(image,180,255,cv2.THRESH_BINARY)
# cv2.imshow('orig',thresh)
# Detect blobs
keypoints = detector.detect(image)

# Draw blobs on the image
blank = np.zeros((1, 1))
for keypoint in keypoints:
    x = int(keypoint.pt[0])
    y = int(keypoint.pt[1])
    image = cv2.circle(image,(x,y),3,(0,0,255),-1)

# print(keypoints)
# blobs = cv2.drawKeypoints(image, keypoints, blank, (0, 0, 255),
#                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show keypoints
cv2.imshow("Blobs", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
