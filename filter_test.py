from sksurgerynditracker.nditracker import NDITracker
import ndicapy
import numpy as np
import numpy.linalg as lin
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfilt,sosfilt_zi
import matplotlib.pyplot as plt
import socket
import datetime
import os

def camera_setup():
    '''
    Fonction fo confiugration of the connection to the camera and initiation of the tracking
    in : none
    out : NDITracker - camera comunication object
    '''
    SETTINGS = {
        "tracker type": "vega",
        "ip address": "169.254.121.249",
        "romfiles" : ["C:\\Files\\Polaris_integration\\markeri\\8700339.rom",
                      "C:\\Files\\Polaris_integration\\markeri\\8700338.rom"],
        
        }
    TRACKER = NDITracker(SETTINGS)
    # TODO set right tracking frequency
    ndicapy.ndiCommand(TRACKER._device,'SET Param.Tracking.Track Frequency=2')
    

    TRACKER.start_tracking()
    return TRACKER

def setup_TCP():
    '''
    Function for configuration of TCP/IP connection with the robot
    in : none
    out : socket
    '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('192.168.40.200',30000))
    print('Connected to server!')
    return client

def avg_filter(data):
    trans = data[0,:3]
    rot = data[0,3:]
    order = data.shape[0]
    for i in range(6):
        for j in range(1,order):
            if i < 3:
                trans[i] += data[j,i]
            else:
                rot[i-3] += data[j,i]
    return trans/order,rot/order


def calculate_distance(tramsforms):
    frame = tramsforms[0]
    target = tramsforms[1]
    tool = tramsforms[2]


    trans_z = np.array([1,0,0,0,0,1,0,0,0,0,1,-40,0,0,0,1])
    trans_z.shape = [4,4]
    target = np.matmul(target,trans_z)

    T_tool = np.matmul(lin.inv(frame),tool)
    T_target = np.matmul(lin.inv(frame),target)

    tool_pos = T_tool[:3,-1]
    target_pos = T_target[:3,-1]
    return tool_pos,target_pos

def calculate_transform(handles,transforms):
    '''
    Function for calculation of transformation between fixed and mobile marker from their transforms in coordinate frame of the camera
    in :    handles - lista prepoznatih markera
            transforms - lista matrica transformcija pojedinih markera u odnosu na koordinatni sustav kamere
    out:    trans - translacijski vektor pomičnog markera u koordinatnom sustavu fiksnog markera
            rot_angles - vektor eulerovih kutova zakreta pomičnog markera u koordinatnom sustavu fiksnog markera, eulerovi kutovi su u redosljedu Z,Y,X kompatibilnim sa KUKA kontrolerom
    '''
    if handles[0] == 1:
        T_st = np.array(transforms[0])
        T_din = np.array(transforms[1])
    else:
        print('a jebiga')
        T_st = np.array(transforms[1])
        T_din = np.array(transforms[0])

    if np.isnan(T_st).any() or np.isnan(T_din).any():
        return np.nan, np.nan
    T_robot = np.matmul(lin.inv(T_st),T_din)

    rot = T_robot[:3,:3]
    r = Rotation.from_matrix(rot)
    rot_angles = r.as_euler('ZYX',degrees=False)

    trans = T_robot[:3,-1]
    

    return trans,rot_angles

def main():


    tracker = camera_setup()
    client = setup_TCP()


    prev_frame = 0
    n = 3*60
    k = 0
    data_buffer = np.zeros([n,6])
   
    
    filter = butter(1,3,btype='lowpass',output='sos',fs=60)
    zi = sosfilt_zi(filter)
    zi = np.dstack((zi,zi,zi,zi,zi,zi))

    msg = client.recv(1024)

    client.setblocking(False)
    client.settimeout(1/120)
    now = datetime.datetime.now()
    os.mkdir('saved_data\\{}_{}_{}'.format(now.date(),now.hour,now.minute))
    for i in range(2):
        data_raw = np.zeros([1,6])
        data_filt = np.zeros([1,6])

        while 1:
            port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
            if framenumbers[0] != prev_frame:

                trans,rot = calculate_transform(port_handles,tracking)
                
                if np.isnan(trans).any() or np.isnan(rot).any():
                    continue
                #data_raw = np.vstack((data_raw,np.hstack((trans,rot))))
                data_buffer = np.vstack((data_buffer,np.hstack((trans,rot))))
                data_buffer = np.delete(data_buffer,0,0)
                if k < n:
                    k += 1
                else:
                    data_raw = np.vstack((data_raw,np.hstack((trans,rot))))

                    #trans,rot = avg_filter(data_buffer)
                    #data_filt = np.vstack((data_filt,np.hstack((trans,rot))))

                    data,zo = sosfilt(filter,data_buffer,axis=0,zi=zi)
                    trans = data[-1,:3]
                    rot = data[-1,3:]
                    data_filt = np.vstack((data_filt,data[-1,:]))

                prev_frame = framenumbers[0]
                try:
                    msg = client.recv(1024)
                    if 'Done' in str(msg):
                        data_raw = np.delete(data_raw,0,0)
                        data_filt = np.delete(data_filt,0,0)
                        data = np.hstack((data_raw,data_filt))
                        #now = datetime.datetime.now()
                        np.savetxt('saved_data\\{}_{}_{}\\movement.csv'.format(now.date(),now.hour,now.minute),data,delimiter=',')
                        plt.figure(1)
                        plt.subplot(3,1,1)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,0],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,0],label='filtered')
                        plt.legend()
                        plt.subplot(3,1,2)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,1],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,1],label='filtered')
                        plt.legend()
                        plt.subplot(3,1,3)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,2],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,2],label='filtered')
                        plt.legend()
                        plt.show()
                    else:
                        data_raw = np.delete(data_raw,0,0)
                        data_filt = np.delete(data_filt,0,0)
                        data = np.hstack((data_raw,data_filt))
                        #now = datetime.datetime.now()
                        np.savetxt('saved_data\\{}_{}_{}\\static.csv'.format(now.date(),now.hour,now.minute),data,delimiter=',')
                        plt.figure(0)
                        plt.subplot(3,1,1)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,0],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,0],label='filtered')
                        plt.legend()
                        plt.subplot(3,1,2)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,1],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,1],label='filtered')
                        plt.legend()
                        plt.subplot(3,1,3)
                        plt.plot(np.arange(len(data_raw)),data_raw[:,2],label='raw')
                        plt.plot(np.arange(len(data_filt)),data_filt[:,2],label='filtered')
                        plt.legend()
                    break

                except TimeoutError:
                    continue


    tracker.stop_tracking()
    
    return



        

if __name__ == '__main__':
    main()