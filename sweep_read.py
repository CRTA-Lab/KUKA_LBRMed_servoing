import socket
import time
import cv2
import numpy as np
from datetime import datetime
import os

def parse_msg(msg):
    msg = str(msg)
    list = msg.split('-')
    angle = list[-1].split('\\')
    #angle = list[0].split('=')[1]
    return angle[0]

os.mkdir("C:/Users/Istrazivac/Desktop/US_pics/sweep_P1_{}_{}".format(datetime.now().hour,datetime.now().minute))

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('172.1.31.100',30001))


cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()
client.send(bytes('next!\n','UTF-8'))
while True:
    msg = client.recv(1024)
    print(msg)
    angle = parse_msg(msg)
    print(angle)
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")

    frame = cv2.resize(frame, (1920, 1080))
    cv2.imshow('slika',frame)
    cv2.waitKey(1)
    cv2.imwrite("C:/Users/Istrazivac/Desktop/US_pics/sweep_P1_{}_{}/slice_{}.jpg".format(datetime.now().hour,datetime.now().minute,angle), frame)
    try:
        client.send(bytes('next!\n','UTF-8'))
    except:
        cap.release()
        break



