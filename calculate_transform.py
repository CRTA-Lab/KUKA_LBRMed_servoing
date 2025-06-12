import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares, differential_evolution
import cv2
from pathlib import Path

#image_path = Path(__file__).parent / "target_5_0.1790.jpg"

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
    lines.append((np.array([-p1[0]*5.5,p1[1]*8,0]),np.array([-p2[0]*5.5,p2[1]*8,-57])))

def noramlize(V):
    ''' Function for normalizing a vector
    Args: V - vector to be normalized (numpy array)
    Returns: V - normalized vector (numpy array)'''
    norm = np.linalg.norm(V)
    if norm == 0: 
       return V
    return V / norm


 
#Eksplicitno korištenje rotacijske matrice + np.matmul() za množenje vektora je zamijenjeno s ugrađenim .apply() iz scipy.spatial.transform.Rotation klase
#Rezultat je isti, no čišće je i brže
def intersection(p1, p2, N, p0, ro):
    u = p2 - p1
    w = p1 - p0
    fac = np.dot(N, w) / np.dot(N, u)
    intersection_global = p1 + fac * u
    intersection_local = intersection_global - p0
    fi = np.arccos(np.dot([0, 0, 1], N) / np.linalg.norm(N))
    rot = Rotation.from_rotvec(noramlize(np.cross([0, 0, 1], N)) * fi)
    intersection_local = rot.apply(intersection_local)
    rot_z = Rotation.from_euler('z', ro)
    return rot_z.apply(intersection_local)

def imread_unicode(path):
    return cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)


def find_blobs_fixed(image):

    #Ucitavamo sliku samo jednom
    #Izbacen je tresh jer si detektirao blobove na originalnoj slici, nije nicemu sluzilo
    #Zadržavamo punu float vrijednost, ne pretvaramo u integer i time gubimo na preciznosti

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
    keypoints = detector.detect(gray)

    blank = np.zeros_like(gray)
    blobs = []
    for k in keypoints:
        x = k.pt[1]      # sub-pixel center (float)
        y = k.pt[0]
        blobs.append((x, y))
        # Draw blob (use int() for pixel position)
        blank = cv2.circle(blank, (int(y), int(x)), 3, 255, -1)

    # Sort blobs
    blobs.sort(key=lambda x: x[0])
    for i in range(0, len(blobs), 3):
        blobs[i:i+3] = sorted(blobs[i:i+3], key=lambda x: x[1])

    return blobs

#image = imread_unicode(image_path)
image = cv2.imread('/home/matija/KUKA_LBRMed_servoing/US_pics/targets/target_5_0.1790.jpg')
blobs = find_blobs_fixed(image)

sigma = 0.5
#sigma je model standardne devijacije pogreške blobsova (koliko očekuješ da će mjerenje biti nepouzdano)
#djeljenjem sa sigma dobiavmp error u standardnim devijacijama i time radimo težnsku optimizaciju
#vrijednost je 0.5 jer blobsovi imaju otprilike ±0.5 px standardnu devijaciju
#original to nije ima pa je svaka točka imala je jednaku težinu i optimizer nije znao kolika je pouzdanost tvog mjerenja
#ako si slučajno imao outlier ili loše detektiran blob to bi jako povuklo rješenje

def calculate_transform(x):
    N, p0, fi = np.array(x[:3]), np.array(x[3:6]), x[6]
    residuals = []
    N_norm = np.linalg.norm(N)
    for (p1, p2), (bx, by) in zip(lines, blobs):
        px, py, _ = intersection(p1, p2, N, p0, fi)
        residuals.append(((px - bx))/ sigma)
        residuals.append(((py - by)) / sigma)
        # if N_norm > 1:
        #     residuals.append(((px - bx) / N_norm)/ sigma)
        #     residuals.append(((py - by) / N_norm) / sigma)
        # else:
        #     residuals.append(((px - bx) * N_norm)/ sigma)
        #     residuals.append(((py - by) * N_norm) / sigma)

    return residuals


#umjesto da rucno zadamo pocetnu tocku differential_evolution() napravimo globalno pretraživanje parametara
#global_result.x postaje pametna startna točka za least_squares()
#puno veća šansa da se nađe pravi globalni minimum i ne zapne u lokalnom
#granice stavljenje dovoljno široko da sonda uvijek upadne u granice, a dovoljno usko da se izbjegne besmisleno traženje ili eksplozija optimizer-a

bounds = [(-0.2, 0.2), (0, 1), (-1, 1), (-50, 100), (-30, 100), (-10, 80), (-np.pi, np.pi)]

global_result = differential_evolution(lambda x: np.sum(np.square(calculate_transform(x))), bounds)
N_candidate = np.array(global_result.x[:3])

result = least_squares(
    calculate_transform, x0=global_result.x,
    bounds=([-0.2, 0, -1, -50, -30, -10, -np.pi],
            [0.2, 1, 1, 100, 100, 80, np.pi]),
    loss='soft_l1', jac='3-point',
    ftol=1e-9, xtol=1e-9, gtol=1e-9 #postrozeni uvjeti zaustavljanja
)

N = np.array(result.x[:3])   # ekstrakcija normale
print(result)                # ispis cijelog least_squares rezultata
print(np.pi/2-np.arccos(np.dot([0, 0, 1], N) / np.linalg.norm(N)))   # kut s Z osi
print(noramlize(N))   # normalizacija