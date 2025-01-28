import cv2

import numpy as np

from random import randint


global thresholded_image
# Function to fit a circle to three points
def fit_circle_three_points(p1, p2, p3, p4):
    A = np.array([
        [p1[0], p1[1], 1],
        [p2[0], p2[1], 1],
        [p3[0], p3[1], 1],
        [p4[0], p4[1], 1],
    ])
    B = np.array([
        -(p1[0]**2 + p1[1]**2),
        -(p2[0]**2 + p2[1]**2),
        -(p3[0]**2 + p3[1]**2),
        -(p4[0]**2 + p4[1]**2),
    ])
    x, y, z = np.linalg.lstsq(A, B, rcond=None)[0]
    a, b = -x / 2, -y / 2
    r = np.sqrt((x**2 + y**2) / 4 - z)
    return a, b, r
 
# RANSAC for circle fitting
def ransac_circle_fitting(points, max_iterations=100, threshold=1.0):
    best_circle = None
    max_inliers = 0
 
    for _ in range(max_iterations):
        # Randomly select 3 points
        sample = points[np.random.choice(points.shape[0], 4, replace=False)]
        try:
            # Fit a circle
            a, b, r = fit_circle_three_points(*sample)
 
            # Count inliers
            distances = np.sqrt((points[:, 0] - a)**2 + (points[:, 1] - b)**2)
            inliers = np.abs(distances - r) < threshold
 
            # Update the best circle
            if np.sum(inliers) > max_inliers:
                max_inliers = np.sum(inliers)
                best_circle = (a, b, r)
 
        except Exception:
            continue
 
    return best_circle
 
# def update_result(*args):
#     global image, gray, thresholded_image
 
#     # Get parameters from trackbars
#     max_iterations = cv2.getTrackbarPos('Max Iterations', 'Controls')
#     threshold = cv2.getTrackbarPos('Threshold', 'Controls') / 10.0
 
#     # Perform RANSAC circle fitting
#     coordinates = np.column_stack(np.where(thresholded_image == 255))
#     if coordinates.shape[0] >= 3:  # Ensure there are enough points
#         result = ransac_circle_fitting(coordinates, max_iterations, threshold)
#         if result:
#             a, b, r = result
#             temp_image = image.copy()
#             cv2.circle(temp_image, (int(b), int(a)), int(r), (0, 255, 0), 2)
#             cv2.imshow("Original", temp_image)
 
 
# Load the image

cap = cv2.VideoCapture(1)
cv2.namedWindow("Controls")

# Initialize Canny edge detection parameters with default values
canny_low_threshold = 50
canny_high_threshold = 150
cv2.createTrackbar("Canny Low Threshold", "Controls", canny_low_threshold, 255, lambda x: None)
cv2.createTrackbar("Canny High Threshold", "Controls", canny_high_threshold, 255, lambda x: None)

# Initialize HoughLinesP parameters with default values
hough_threshold = 50
cv2.createTrackbar("Hough Threshold", "Controls", hough_threshold, 255, lambda x: None)

#cv2.namedWindow("Controls")
cv2.createTrackbar('Max Iterations', 'Controls', 100, 500, lambda x:x)
cv2.createTrackbar('Threshold', 'Controls', 10, 100, lambda x:x)
cv2.createTrackbar('Threshold2', 'Controls', 10, 255, lambda x:x)


i = 1

boje = [(randint(0,255),randint(0,255),randint(0,255)) for i in range(10)]

while True:

    ret,frame = cap.read()
    frame = cv2.resize(frame, (1920, 1080))
    image = frame[125:720, 645:1345]


    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    thresh2 = cv2.getTrackbarPos("Threshold2", "Controls")


    _, thresholded_image = cv2.threshold(gray, 95, 255, cv2.THRESH_BINARY)
    #blurred = cv2.medianBlur(gray, 3)
    # blurred = cv2.GaussianBlur(gray,(7,7),5)

    canny_low_threshold = cv2.getTrackbarPos("Canny Low Threshold", "Controls")
    canny_high_threshold = cv2.getTrackbarPos("Canny High Threshold", "Controls")
    
    # Detect edges

    #thresh = cv2.getTrackbarPos("Threshold", "Controls")
    #iterations = cv2.getTrackbarPos("Max Iterations", "Controls")

    edges = cv2.Canny(thresholded_image, canny_low_threshold, canny_high_threshold, apertureSize=3)

    # coordinates = np.column_stack(np.where(edges == 255))
    # if coordinates.shape[0] >= 3:  # Ensure there are enough points
    #     result = ransac_circle_fitting(coordinates, iterations, thresh)
    #     if result:
    #         a, b, r = result

    

    hough_threshold = cv2.getTrackbarPos("Hough Threshold", "Controls")

    
    #Detect lines using Hough Line Transform

    lines = cv2.HoughLines(edges, 1, np.pi / 180, 50,max_theta=np.pi,min_theta=0)

    #print(lines[:, 0])
    if lines is not None:

        lines_mat = lines[:, 0]
        #print(lines_mat.shape)
        lines_mat = lines_mat[lines_mat[:,1].argsort()]
        #print(lines_mat)
        differences = np.diff(lines_mat[:,1])
        #print(differences)

        # Step 3: Define a threshold for significant difference (you can adjust this value)
        threshold = 10*np.pi/180

        # Step 4: Identify the boundaries where the difference exceeds the threshold
        boundaries = np.where(np.abs(differences) > threshold)[0] + 1  # Add 1 because np.diff reduces length by 1

        # Step 5: Assign group labels
        group_labels = np.zeros(len(lines_mat))
        group_id = 0
        for i in range(1, len(lines_mat)):
            if i in boundaries:
                group_id += 1
            group_labels[i] = group_id
        #print(group_labels)
    

        # Convert polar coordinates to Cartesian and collect endpoints

        line_coords = []

        for rho, theta in lines_mat:

            a = np.cos(theta)

            b = np.sin(theta)

            x0 = a * rho

            y0 = b * rho

            x1 = int(x0 + 1000 * (-b))

            y1 = int(y0 + 1000 * a)

            x2 = int(x0 - 1000 * (-b))

            y2 = int(y0 - 1000 * a)

            line_coords.append(((x1, y1), (x2, y2)))
        
        # Draw lines and find intersection
        def line_intersection(line1, line2):

            x1, y1, x2, y2 = *line1[0], *line1[1]

            x3, y3, x4, y4 = *line2[0], *line2[1]

            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

            if denom == 0:

                return None  # Lines are parallel

            px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom

            py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

            return int(px), int(py)
    
        #Assuming the first two lines are the stylus edges
        _means = [np.array([0,0]) for i in range(int(group_labels[-1])+1)]
        try:

            intersection = line_intersection(line_coords[0], line_coords[1])

        
        # Draw the lines and intersection            
            
            for i in range(len(line_coords)):
                line = line_coords[i]

                #cv2.line(image, line[0], line[1], boje[int(group_labels[i])], 1)
                _means[int(group_labels[i])] = np.vstack((_means[int(group_labels[i])],lines_mat[i,:]))
            means = []
            for i in _means:
                i = np.delete(i,0,0)
                means.append(np.mean(i,axis=0,dtype=np.float64))
            #print(means)
            
            has_ortog = [0 for i in range(int(group_labels[-1])+1)]
            for i in range(len(means)):
                for j in means:
                    if abs(min(abs(means[i][1] - j[1]),2*np.pi-abs(means[i][1] - j[1]))-np.pi/2) < 7*np.pi/180:
                        has_ortog[i] = 1
                        print(min(abs(means[i][1] - j[1]),2*np.pi-abs(means[i][1] - j[1]))*180/np.pi)
            i = 0
            for rho, theta in means:

                a = np.cos(theta)

                b = np.sin(theta)

                x0 = a * rho

                y0 = b * rho

                x1 = int(x0 + 1000 * (-b))

                y1 = int(y0 + 1000 * a)

                x2 = int(x0 - 1000 * (-b))

                y2 = int(y0 - 1000 * a)

                #if has_ortog[i]:

                #cv2.line(image, (x1, y1), (x2, y2), (0,0,255), 3)
                i += 1
                # line_coords.append(((x1, y1), (x2, y2)))
            if intersection:

                cv2.circle(image, intersection, 5, (0, 0, 255), -1)
        
        except RuntimeError as e:
            print('Dogodila se greška:\n{}'.format(e))
    
    # Show the result
    # cv2.line(image, (image.shape[1]//2, 0), (image.shape[1]//2, image.shape[0]), (0,0,255), 2)
    # cv2.line(image, (image.shape[1]-150, 0), (image.shape[1]-150, image.shape[0]), (0,0,255), 2)
    # cv2.line(image, (150, 0), (150, image.shape[0]), (0,0,255), 2)

    # cv2.line(image, (0,150), (image.shape[1], 150), (0,0,255), 2)
    # cv2.line(image, (0,image.shape[0]-150), (image.shape[1], image.shape[0]-150), (0,0,255), 2)

    cv2.line(image, (image.shape[1]//2, 0), (image.shape[1]//2, image.shape[0]), (0,0,255), 2)
    cv2.line(image, (0,image.shape[0]//2), (image.shape[1], image.shape[0]//2), (0,0,255), 2)

    #cv2.circle(image,(int(b),int(a)),int(r),(0,255,0),2)

    cv2.imshow('Stylus with Intersection', image)
    cv2.imshow('edges',edges)

    if cv2.waitKey(1) & 0xFF == ord('p'):
        cv2.imwrite("C:/Users/Istrazivac/Desktop/US_pics/conus/" + str(i) + ".jpg", image)
        i += 1


    cv2.waitKey(1)