import socket
import struct
import time
import cv2
import numpy as np
from datetime import datetime
import os

def parse_msg(msg):
    msg = str(msg)
    list = msg.split(' ')
    angle = list[0].split('=')
    #angle = list[0].split('=')[1]
    return angle[1]


def receive_img(client):
    client.send(b'a')
    total_bytes = client.recv(4)
    if not total_bytes:
        return
    total_bytes = struct.unpack('<I', total_bytes)[0]
    header_bytes = client.recv(40)
    if not header_bytes:
        return
    pixel_bytes = client.recv(total_bytes - 40)
    while len(pixel_bytes) < (total_bytes - 40):
        more_bytes = client.recv((total_bytes - 40) - len(pixel_bytes))
        if not more_bytes:
            break
        pixel_bytes += more_bytes
    if not pixel_bytes:
        return
    file_size = 14+total_bytes
    bmp_file_header = struct.pack('<2sIHHI', b'BM', file_size, 0, 0, 14+40)
    bmp_bytes = bmp_file_header + header_bytes + pixel_bytes
    frame = np.frombuffer(bmp_bytes, dtype=np.uint8)
    frame = cv2.imdecode(frame, cv2.IMREAD_GRAYSCALE)
    return frame

save_dir = "/home/racunalo/KUKA_LBRMed_servoing/US_pics/sweep_P1_{}_{}".format(datetime.now().hour,datetime.now().minute)
os.mkdir(save_dir)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('192.168.0.100',30001))

client_US = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_US.connect(('192.168.0.168',9999))


# cap = cv2.VideoCapture(0)

# if not cap.isOpened():
#     print("Error: Could not open video device.")
#     exit()

while True:
    frame = receive_img(client_US)
    client.send(bytes('next!\n','UTF-8'))
    msg = client.recv(1024)
    print(msg)
    angle = parse_msg(msg)
    print(angle)
    # ret, frame = cap.read()
    # if not ret:
    #     print("Error: Could not read frame.")

    # frame = cv2.resize(frame, (1920, 1080))
    # frame = frame[125:720, 645:1345]
    cv2.imshow('slika',frame)
    cv2.waitKey(1)
    cv2.imwrite("{}/slice_{}.jpg".format(save_dir, angle), frame)
    try:
        client.send(bytes('next!\n','UTF-8'))
    except:
        # cap.release()
        break



