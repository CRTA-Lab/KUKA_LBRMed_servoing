import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import sympy
import cv2

# points defineing lines on the phantom
# (x,y) coordinates are in hole coordinates
points = [((3,0),(3,0)),
          ((7,0),(5,0)),
          ((8,0),(8,0)),
          ((9,1),(9,1)),
          ((2,1),(8,1)),
          ((1,1),(1,1)),
          ((8,2),(8,2)),
          ((7,2),(1,2)),
          ((0,2),(0,2)),
          ((7,3),(7,3)),
          ((3,3),(6,3)),
          ((2,3),(2,3)),
          ((8,4),(8,4)),
          ((7,4),(4,4)),
          ((3,4),(3,4)),
          ((9,5),(9,5)),
          ((2,5),(8,5)),
          ((0,5),(0,5)),
          ((7,6),(7,6)),
          ((6,6),(2,6)),
          ((1,6),(1,6)),
          ((8,7),(8,7)),
          ((3,7),(7,7)),
          ((2,7),(2,7))]

global lines
lines = []
# transform points from hole coordinates to milmeters
for (p1,p2) in points:
    lines.append((np.array([p1[0]*5.5,p1[1]*8,0]),np.array([p2[0]*5.5,p2[1]*8,57])))


def noramlize(V):
    ''' Function for normalizing a vector
    Args: V - vector to be normalized (numpy array)
    Returns: V - normalized vector (numpy array)'''
    norm = np.linalg.norm(V)
    if norm == 0: 
       return V
    return V / norm

def intersection(p1,p2,N,p0,ro):
    ''' Function for calculating intersection of line and plane in 3D space
    Created according to answer to https://stackoverflow.com/questions/5666222/3d-line-plane-intersection by user ideasman42
    Args: p1 - first point of the line (numpy array)
          p2 - second point of the line (numpy array)
          N - normal of the plane (numpy array)
          p0 - point on the plane (numpy array)
          ro - rotation angle around z axis (float)
    Returns: intersection_local - intersection point in local coordinates (numpy array)'''
    u  = p2 - p1
    w  = p1 - p0
    fac = np.dot(N,w) / np.dot(N,u)
    intersection_global = p1 + fac*u
    intersection_local = intersection_global - p0
    fi = np.arccos((np.dot(np.array([0,0,1]),N))/(np.linalg.norm(N)))
    rot = Rotation.from_rotvec(noramlize(np.cross(np.array([0,0,1]),N))*fi)
    rot_mat = rot.as_matrix()
    intersection_local = np.matmul(rot_mat,intersection_local)
    rot = Rotation.from_euler('z',ro)
    rot_mat = rot.as_matrix()
    intersection_local = np.matmul(rot_mat,intersection_local)
    return intersection_local

def find_blobs():
    ''' Function for finding blobs in the image
    Args: None
    Returns: blobs - list of blobs (list of tuples)'''
    image = cv2.imread('/home/matija/KUKA_LBRMed_servoing/US_pics/targets/target_1_-0.1098.jpg')
    #cv2.imshow('orig',image)

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
    #cv2.imshow('tresh',thresh)


    # Detect blobs
    keypoints = detector.detect(image)

    blank = np.zeros(np.shape(gray))

    blobs = []

    for keypoint in keypoints:
        x = int(keypoint.pt[1])
        y = int(keypoint.pt[0])
        blobs.append((x,y))
        blank = cv2.circle(blank,(y,x),3,255,-1)
    #cv2.imwrite(self.img_path + '/output_images/slice_{:.5f}.jpg'.format(angle),blank)
    #cv2.imshow('blobs',blank)
    #cv2.waitKey(0)
    blobs.sort(key=lambda x: x[0])
    for i in range(0,len(blobs),3):
        blobs[i:i+3] = sorted(blobs[i:i+3],key=lambda x: x[1])
    #print(blobs)
    return blobs

def calculate_transform(x):
    ''' Function to be used in the least squares optimization, return the sum of the differences between the calculated and the real points
    Args: x - vector of parameters (numpy array)
    Returns: err - sum of the differences scaled by  (float)'''
    global lines
    N = np.array(x[0:3])
    p0 = np.array(x[3:6])
    fi = x[6]
    blobs = find_blobs()
    diffs = []
    for i in range(len(lines)):
        point = intersection(lines[i][0],lines[i][1],N,p0,fi)
        diffs.append(np.sqrt((point[0]-blobs[i][0])**2+(point[1]-blobs[i][1])**2))
        # if np.linalg.norm(N) < 1:
        #     diffs.append(np.sqrt((point[0]-blobs[i][0])**2+(point[1]-blobs[i][1])**2)/np.linalg.norm(N))
        # else:
        #     diffs.append(np.sqrt((point[0]-blobs[i][0])**2+(point[1]-blobs[i][1])**2)*np.linalg.norm(N))
    if np.linalg.norm(N) < 1:
        return np.sum(diffs)/np.linalg.norm(N)
    else:
        return np.sum(diffs)*np.linalg.norm(N)
    #return diffs


result = least_squares(calculate_transform, x0=[0,0,1,-5,-5,28,0],bounds=([-0.2,-1,-1,-100,-100,-100,-np.pi],[0.2,1,1,100,100,100,np.pi]),loss='soft_l1',jac='3-point',)
#print(result.x)
print(result)
N = np.array(result.x[0:3])

# Calcualting the angle between the normal and the z axis, which is the angle giver to the robot for aiming
print(np.arccos((np.dot(np.array([0,0,1]),N))/(np.linalg.norm(N))))
